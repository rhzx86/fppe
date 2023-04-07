use std::num::NonZeroU16;

use bitflags::bitflags;
use fixed::{traits::ToFixed, types::I32F32};
pub use vec::Vec3;

mod vec;

pub type Unit = I32F32;

fn non_zero(unit: Unit) -> Unit {
    if unit == 0 {
        1.into()
    } else {
        unit
    }
}

/// Maximum number of iterations to try to uncollide two colliding bodies.
const COLLISION_RESOLUTION_ITERATIONS: usize = 16;

// Margin by which a body will be shifted back to get out of collision.
const COLLISION_RESOLUTION_MARGIN: Unit =
    Unit::from_bits(0b0000000000000000000000000000000000000100000000000000000000000000); // 1/64

/// Number of times a collision of nonrotating bodies with environment will be
/// attempted to resolve. This probably won't have great performance implications
/// as complex collisions of this kind should be relatively rare.
const NONROTATING_COLLISION_RESOLVE_ATTEMPTS: usize = 8;

/// Limit within which acceleration caused by connection tension won't be
/// applied.
const TENSION_ACCELERATION_THRESHOLD: usize = 5;

/// Connection tension threshold after which twice as much acceleration will
/// be applied. This helps prevent diverting joints that are "impaled" by
/// environment.
const TENSION_GREATER_ACCELERATION_THRESHOLD: usize = TENSION_ACCELERATION_THRESHOLD * 3;

/// Number by which the base acceleration (TPE_FRACTIONS_PER_UNIT per tick
/// squared) caused by the connection tension will be divided. This should be
/// power of 2.
pub const TENSION_ACCELERATION_DIVIDER: Unit =
    Unit::from_bits(0b0000000000000000000000000000000000010000000000000000000000000000); // 1/64

// Tension limit, in TPE_Units, after which a non-soft body will be reshaped.
// Smaller number will keep more stable shapes but will cost more performance.
pub const RESHAPE_TENSION_LIMIT: Unit =
    Unit::from_bits(0b0000000000000000000000000000000000001010001111010111000010100011); // 1 / 25

/// How many iterations of reshaping will be performed by the step function if
/// the body's shape needs to be reshaped. Greater number will keep shapes more
/// stable but will cost some performance.
const RESHAPE_ITERATIONS: usize = 3;

/// After how many ticks of low speed should a body be disabled. This mustn't
/// be greater than 255.
const DEACTIVATE_AFTER: u8 = 128;

/// When a body is activated by a collision, its deactivation counter will be
/// set to this value, i.e. after a collision the body will be prone to deactivate
/// sooner than normally. This is to handle situations with many bodies touching
/// each other that would normally keep activating each other, never coming to
/// rest.
const LIGHT_DEACTIVATION: u8 = DEACTIVATE_AFTER - DEACTIVATE_AFTER / 10;

/// Speed, in TPE_Units per ticks, that is considered low (used e.g. for auto
/// deactivation of bodies).
pub const LOW_SPEED: Unit =
    Unit::from_bits(0b0000000000000000000000000000000000010001000100010001000100010001); // physics step 1/60

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct Joint {
    pub position: Vec3,
    velocity: Vec3,
    size: Unit,
}

impl Joint {
    pub fn new<T: Into<Unit>>(position: Vec3, size: T) -> Self {
        Self {
            position,
            velocity: Default::default(),
            size: size.into(),
        }
    }

    pub fn environment_resolve_collision(
        &mut self,
        elasticity: Unit,
        friction: Unit,
        closest_env_point: ClosestPointFn,
    ) -> u8 {
        let mut to_joint = self.position - closest_env_point(self.position, self.size);
        let mut len = to_joint.length();

        if len < self.size {
            let position_backup = self.position;
            let mut shift = Vec3::default();
            let mut success = false;

            if len > 0 {
                // Joint center is still outside the geometry so we can
                // determine the normal and use it to shift it outside. This can
                // still leave the joint colliding though, so try to repeat it a
                // few times.

                for _ in 0..COLLISION_RESOLUTION_ITERATIONS {
                    shift = to_joint.normalize();
                    shift = shift * (self.size - len + COLLISION_RESOLUTION_MARGIN);

                    self.position += shift;

                    to_joint = self.position - closest_env_point(self.position, self.size);

                    len = to_joint.length();

                    if len > self.size {
                        success = true;
                        break;
                    }
                }
            }

            if !success {
                // Shifting along normal was unsuccessfull, now try different
                // approach: shift back by joint velocity.

                shift = Vec3::new(
                    -1 * self.velocity.x,
                    -1 * self.velocity.y,
                    -1 * self.velocity.z,
                );

                for _ in 0..COLLISION_RESOLUTION_ITERATIONS {
                    self.position += shift;

                    to_joint = self.position - closest_env_point(self.position, self.size);

                    len = to_joint.length(); // still colliding?

                    if len >= self.size {
                        success = true;
                        break;
                    }

                    shift.x /= 2; // decrease the step a bit
                    shift.y /= 2;
                    shift.z /= 2;
                }
            }

            if success {
                let vel = self.velocity.project_onto(shift); // parallel part of velocity
                let vel2 = self.velocity - vel; // perpendicular part of velocity

                let vel2 = vel2 * friction;
                let vel = vel * (Unit::from_num(1) + elasticity);

                self.velocity.x -= vel.x + vel2.x;
                self.velocity.y -= vel.y + vel2.y;
                self.velocity.z -= vel.z + vel2.z;
            } else {
                eprintln!("WARNING: joint-environment collision couldn't be resolved");

                self.position = position_backup;
                self.velocity.x = 0.into();
                self.velocity.y = 0.into();
                self.velocity.z = 0.into();

                return 2;
            }
        }

        // not collided
        return 0;
    }

    fn resolve_collision_with_other_joint(
        joint1: &mut Self,
        joint2: &mut Self,
        mass1: Unit,
        mass2: Unit,
        elasticity: Unit,
        friction: Unit,
        closest_env_point: ClosestPointFn,
    ) -> bool {
        let dir = joint2.position - joint1.position;
        let d = dir.length() - joint1.size - joint2.size;

        if d < 0 {
            let pos1_backup = joint1.position;
            let pos2_backup = joint2.position;

            // separate joints, the shift distance will depend on the weight ratio:

            let d = -1 * d * COLLISION_RESOLUTION_MARGIN;

            let dir = dir.normalize();

            let ratio = mass2 / (mass1 + mass2);

            let shift_distance = ratio * d;

            let shift = dir * shift_distance;

            joint1.position += shift;

            // compute new velocities

            let vel = joint1.velocity.project_onto(dir);
            joint1.velocity -= vel;

            /* friction explanation: Not physically correct (doesn't depend on load),
            friction basically means we weighted average the velocities of the bodies
            in the direction perpendicular to the hit normal, in the ratio of their
            masses, friction coefficient just says how much of this effect we apply
            (it multiplies the friction vectors we are subtracting) */

            let friction_vec = joint1.velocity;

            let mut v1 = vel.dot(dir);

            let vel = joint2.velocity.project_onto(dir);
            joint2.velocity -= vel;

            let friction_vec = joint2.velocity - friction_vec;

            let mut v2 = vel.dot(dir);

            Joint::velocities_after_collision(&mut v1, &mut v2, mass1, mass2, elasticity);

            {
                let vel = dir * v1;
                joint1.velocity = joint1.velocity + vel + ((friction_vec * ratio) * friction);

                let vel = dir * v2;
                let ratio = Unit::ONE - ratio; // is this correct ?
                joint2.velocity = joint2.velocity + vel - ((friction_vec * ratio) * friction);
            }

            // ensure the joints aren't colliding with environment

            // TODO: make closes_env_point optional

            if joint1.environment_resolve_collision(elasticity, friction, closest_env_point) == 2 {
                joint1.position = pos1_backup;
            }

            if joint2.environment_resolve_collision(elasticity, friction, closest_env_point) == 2 {
                joint2.position = pos2_backup;
            }

            return true;
        }

        return false;
    }

    fn velocities_after_collision(
        v1: &mut Unit,
        v2: &mut Unit,
        m1: Unit,
        m2: Unit,
        elasticity: Unit,
    ) {
        /* In the following a lot of TPE_F cancel out, feel free to
        check if confused. */

        let m1_plus_m2 = non_zero(m1 + m2);
        let v2_minus_v1 = non_zero(*v2 - *v1);

        let m1v1_plus_m2v2 = (m1 * *v1) + (m2 * *v2);

        *v1 = (((elasticity * m2) * v2_minus_v1) + m1v1_plus_m2v2) / m1_plus_m2;

        *v2 = (((elasticity * m1) * -1 * v2_minus_v1) + m1v1_plus_m2v2) / m1_plus_m2;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Connection {
    /// joint1 index
    joint1: u8,

    /// joint2 index
    joint2: u8,

    /// connection's preferred length
    length: NonZeroU16,
}

impl Default for Connection {
    fn default() -> Self {
        Self {
            joint1: Default::default(),
            joint2: Default::default(),
            length: NonZeroU16::new(1).unwrap(),
        }
    }
}

impl Connection {
    pub fn new(joint1: u8, joint2: u8) -> Self {
        Self {
            joint1,
            joint2,
            length: NonZeroU16::new(1).unwrap(),
        }
    }
}

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
    struct BodyFlags: u8 {
        /// Not being updated due to low energy, "sleeping", will be woken by
        /// collisions etc.
        const DEACTIVATED   = 0b00000001;

        /// When set, the body won't rotate, will only move linearly. Here the
        /// velocity of the body's first joint is the velocity of the whole
        /// body.
        const NONROTATING   = 0b00000010;

        /// Disabled, not taking part in simulation.
        const DISABLED      = 0b00000100;

        /// Soft connections, effort won't be made to keep the body's shape.
        const SOFT          = 0b00001000;

        /// Simple connections, don't zero out antagonist forces or apply
        /// connection friction, can increase performance.
        const SIMPLE_CONN   = 0b00010000;

        /// Will never deactivate due to low energy.
        const ALWAYS_ACTIVE = 0b00100000;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct BoundingBox {
    pub min: Vec3,
    pub max: Vec3,
}

impl BoundingBox {
    fn overlaps_with(&self, other: &BoundingBox) -> bool {
        let dist = (self.min.x + self.max.x - other.max.x - other.min.x).abs();
        if dist > self.max.x - self.min.x + other.max.x - other.min.x {
            return false;
        }

        let dist = (self.min.y + self.max.y - other.max.y - other.min.y).abs();
        if dist > self.max.y - self.min.y + other.max.y - other.min.y {
            return false;
        }

        let dist = (self.min.z + self.max.z - other.max.z - other.min.z).abs();
        if dist > self.max.z - self.min.z + other.max.z - other.min.z {
            return false;
        }

        return true;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct BoundingSphere {
    pub center: Vec3,
    pub radius: Unit,
}

/// Physics body made of spheres (each of same weight but possibly different
/// radia) connected by elastic springs.
#[derive(Default, Debug, Hash)]
pub struct Body {
    pub joints: Vec<Joint>,
    connections: Vec<Connection>,
    joint_mass: Unit,
    ///< mass of a single joint
    friction: Unit,
    ///< friction of each joint
    elasticity: Unit,
    ///< elasticity of each joint
    flags: BodyFlags,
    deactivate_count: u8,
}

impl Body {
    pub fn new<T: Into<Unit>>(joints: Vec<Joint>, connections: Vec<Connection>, mass: T) -> Self {
        assert!(joints.len() > 0, "body must have at least one joint");

        let joints_len = Unit::from_num(joints.len());

        Self {
            joints,
            connections,
            joint_mass: joints_len / mass.into(),
            friction: 0.5.to_fixed(),
            elasticity: 0.5.to_fixed(),
            ..Default::default()
        }
    }

    pub fn is_active(&self) -> bool {
        !self.flags.contains(BodyFlags::DEACTIVATED)
    }

    pub fn center_of_mass(&self) -> Vec3 {
        // note that joint sizes don't play a role as all weight the same

        let mut result = Vec3::default();

        for joint in &self.joints {
            result += joint.position;
        }

        let joint_count = Unit::from_num(self.joints.len());

        result.x /= joint_count;
        result.y /= joint_count;
        result.z /= joint_count;

        result
    }

    pub fn apply_gravity(&mut self, downwards_acceleration: Unit) {
        if self.flags.contains(BodyFlags::DEACTIVATED) || self.flags.contains(BodyFlags::DISABLED) {
            return;
        }

        for joint in &mut self.joints {
            joint.velocity.y -= downwards_acceleration;
        }
    }

    pub fn compute_aabb(&self) -> BoundingBox {
        let first_joint = self.joints.first().unwrap();

        let mut aabb = BoundingBox {
            min: first_joint.position - first_joint.size,
            max: first_joint.position + first_joint.size,
        };

        for joint in self.joints.iter().skip(1) {
            let min = joint.position - joint.size;
            let max = joint.position + joint.size;

            // x
            {
                if min.x < aabb.min.x {
                    aabb.min.x = min.x;
                }

                if max.x > aabb.max.x {
                    aabb.max.x = max.x;
                }
            }

            // y
            {
                if min.y < aabb.min.y {
                    aabb.min.y = min.y;
                }

                if max.y > aabb.max.y {
                    aabb.max.y = max.y;
                }
            }

            // z
            {
                if min.z < aabb.min.z {
                    aabb.min.z = min.z;
                }

                if max.z > aabb.max.z {
                    aabb.max.z = max.z;
                }
            }
        }

        aabb
    }

    // Computes sphere from bounding box, which is fast but not that accurate.
    pub fn compute_fast_bounding_sphere(&self) -> BoundingSphere {
        let aabb = self.compute_aabb();

        let mut center = Vec3::default();
        center.x = (aabb.min.x + aabb.max.x) / 2;
        center.y = (aabb.min.y + aabb.max.y) / 2;
        center.z = (aabb.min.z + aabb.max.z) / 2;

        let radius = center.distance(aabb.max);

        BoundingSphere { center, radius }
    }

    pub fn environment_resolve_collision(&mut self, closest_env_point: ClosestPointFn) -> bool {
        let bsphere = self.compute_fast_bounding_sphere();

        let closest_point = closest_env_point(bsphere.center, bsphere.radius);

        if bsphere.center.distance(closest_point) > bsphere.radius {
            return false;
        }

        // now test the full body collision:

        let mut collision = false;

        for i in 0..self.joints.len() {
            let (before, joint, after) = split_slice_before_after(&mut self.joints, i);
            let previous_pos = joint.position;

            let ret = joint.environment_resolve_collision(
                self.elasticity,
                self.friction,
                closest_env_point,
            );

            if ret != 0 {
                collision = true;
                Self::nonrotating_joint_collided(before, previous_pos, joint, ret == 2);
                Self::nonrotating_joint_collided(after, previous_pos, joint, ret == 2);
            }
        }

        collision
    }

    fn nonrotating_joint_collided(
        other_joints: &mut [Joint],
        collided_joint_orig_pos: Vec3,
        collided_joint: &Joint,
        success: bool,
    ) {
        let orig_pos = collided_joint.position - collided_joint_orig_pos;

        for joint in other_joints {
            joint.position += orig_pos;

            if success {
                joint.velocity = collided_joint.velocity;
            }
        }
    }

    fn environment_collide(&self, closest_env_point: ClosestPointFn) -> bool {
        for joint in &self.joints {
            let distance_to_env = joint
                .position
                .distance(closest_env_point(joint.position, joint.size));

            if distance_to_env <= joint.size {
                return true;
            }
        }

        return false;
    }

    fn move_by(&mut self, offset: Vec3) {
        for joint in &mut self.joints {
            joint.position += offset;
        }
    }

    pub fn move_to(&mut self, position: Vec3) {
        let position = position - self.center_of_mass();

        for joint in &mut self.joints {
            joint.position += position;
        }
    }

    fn reshape(&mut self, closest_env_point: ClosestPointFn) {
        for connection in &self.connections {
            let (joint1, joint2) = get_pair_mut(
                &mut self.joints,
                connection.joint1 as usize,
                connection.joint2 as usize,
            )
            .unwrap();

            let dir = (joint2.position + joint1.position).normalize();
            let middle = (joint1.position + joint2.position) / Unit::from_num(2);

            let dir = dir * connection.length.get().into();

            let position_backup = joint1.position;

            joint1.position.x = middle.x - dir.x / 2;
            joint1.position.y = middle.y - dir.y / 2;
            joint1.position.z = middle.z - dir.z / 2;

            if (joint1.position - closest_env_point(joint1.position, joint1.size)).length()
                < joint1.size
            {
                joint1.position = position_backup;
            }

            let position_backup = joint2.position;

            joint2.position.x = middle.x + dir.x / 2;
            joint2.position.y = middle.y + dir.y / 2;
            joint2.position.z = middle.z + dir.z / 2;

            if (joint2.position - closest_env_point(joint2.position, joint2.size)).length()
                < joint2.size
            {
                joint2.position = position_backup;
            }
        }
    }

    fn cancel_out_velocities(&mut self, strong: bool) {
        for connection in &self.connections {
            let (joint1, joint2) = get_pair_mut(
                &mut self.joints,
                connection.joint1 as usize,
                connection.joint2 as usize,
            )
            .unwrap();

            let dir = joint2.position - joint1.position;

            let length = dir.length();
            let length = if length == 0 { 1.into() } else { length };

            let mut cancel = true;

            if strong {
                let tension = connection_tension(length, connection.length.get().into());
                cancel = tension.abs() >= TENSION_ACCELERATION_THRESHOLD;
            }

            if cancel {
                let dir = dir.normalize();

                let v1 = joint1.velocity.project_onto_normalized(dir);
                let v2 = joint2.velocity.project_onto_normalized(dir);
                let avg = (v1 + v2) / 2.into();

                if strong {
                    joint1.velocity.x = joint1.velocity.x - v1.x + avg.x;
                    joint1.velocity.y = joint1.velocity.y - v1.y + avg.y;
                    joint1.velocity.z = joint1.velocity.z - v1.z + avg.z;

                    joint2.velocity.x = joint2.velocity.x - v2.x + avg.x;
                    joint2.velocity.y = joint2.velocity.y - v2.y + avg.y;
                    joint2.velocity.z = joint2.velocity.z - v2.z + avg.z;
                } else {
                    joint1.velocity.x = joint1.velocity.x - v1.x + (v1.x * 3 + avg.x) / 4;
                    joint1.velocity.y = joint1.velocity.y - v1.y + (v1.y * 3 + avg.y) / 4;
                    joint1.velocity.z = joint1.velocity.z - v1.z + (v1.z * 3 + avg.z) / 4;

                    joint2.velocity.x = joint2.velocity.x - v2.x + (v2.x * 3 + avg.x) / 4;
                    joint2.velocity.y = joint2.velocity.y - v2.y + (v2.y * 3 + avg.y) / 4;
                    joint2.velocity.z = joint2.velocity.z - v2.z + (v2.z * 3 + avg.z) / 4;
                }
            }
        }
    }

    fn resolve_collision_with_other_body(
        &mut self,
        other: &mut Body,
        closest_env_point: ClosestPointFn,
    ) -> bool {
        let mut collided = false;

        for i in 0..self.joints.len() {
            let (joint1_before, joint1, joint1_after) =
                split_slice_before_after(&mut self.joints, i);

            for j in 0..other.joints.len() {
                let (joint2_before, joint2, joint2_after) =
                    split_slice_before_after(&mut other.joints, j);

                let orig_pos_1 = joint1.position;
                let orig_pos_2 = joint2.position;

                let joints_collided = Joint::resolve_collision_with_other_joint(
                    joint1,
                    joint2,
                    self.joint_mass,
                    other.joint_mass,
                    (self.elasticity + other.elasticity) / 2,
                    (self.friction + other.friction) / 2,
                    closest_env_point,
                );

                if joints_collided {
                    collided = true;

                    if self.flags.contains(BodyFlags::NONROTATING) {
                        Self::nonrotating_joint_collided(joint1_before, orig_pos_1, joint1, true);
                        Self::nonrotating_joint_collided(joint1_after, orig_pos_1, joint1, true);
                    }

                    if other.flags.contains(BodyFlags::NONROTATING) {
                        Self::nonrotating_joint_collided(joint2_before, orig_pos_2, joint2, true);
                        Self::nonrotating_joint_collided(joint2_after, orig_pos_2, joint2, true);
                    }
                }
            }
        }

        collided
    }

    fn stop(&mut self) {
        for joint in &mut self.joints {
            joint.velocity = Default::default();
        }
    }

    fn activate(&mut self) {
        // the if check has to be here, don't remove it

        if self.flags.contains(BodyFlags::DEACTIVATED) {
            self.stop();
            self.flags.remove(BodyFlags::DEACTIVATED);
            self.deactivate_count = 0;
        }
    }

    fn net_speed(&self) -> Unit {
        let mut velocity = Unit::ZERO;

        for joint in &self.joints {
            velocity += joint.velocity.length();
        }

        return velocity;
    }

    fn average_speed(&self) -> Unit {
        self.net_speed() / Unit::from_num(self.joints.len())
    }

    pub fn accelerate(&mut self, velocity: Vec3) {
        self.activate();

        for joint in self.joints.iter_mut() {
            joint.velocity += velocity;
        }
    }

    pub fn spin_with_center(&mut self, rotation: Vec3, center: Vec3) {
        for joint in self.joints.iter_mut() {
            let to_point = joint.position - center;
            let to_point = to_point.project_onto(rotation);
            let to_point = center + to_point;
            let to_point = joint.position - to_point;
            let to_point = to_point.cross(rotation);

            joint.velocity += to_point;
        }
    }

    pub fn spin(&mut self, rotation: Vec3) {
        self.spin_with_center(rotation, self.center_of_mass());
    }
}

impl Body {
    // #define C(n,a,b) connections[n].joint1 = a; connections[n].joint2 = b;

    pub fn make_box<T: Into<Unit>>(width: T, depth: T, height: T, joint_size: T, mass: T) -> Body {
        let mut joints: [Joint; 8] = Default::default();
        let mut connections: [Connection; 16] = Default::default();

        let width = width.into() / 2;
        let depth = depth.into() / 2;
        let height = height.into() / 2;
        let joint_size = joint_size.into();

        for i in 0..joints.len() {
            joints[i] = Joint::new(
                Vec3::new(
                    if i % 2 != 0 { width } else { -1 * width },
                    if (i >> 2) % 2 != 0 {
                        height
                    } else {
                        -1 * height
                    },
                    if i % 2 != 0 { depth } else { -1 * depth },
                ),
                joint_size,
            );
        }

        // top
        connections[0] = Connection::new(0, 1);
        connections[1] = Connection::new(1, 3);
        connections[2] = Connection::new(3, 2);
        connections[3] = Connection::new(2, 0);

        // bottom
        connections[4] = Connection::new(4, 5);
        connections[5] = Connection::new(5, 7);
        connections[6] = Connection::new(7, 6);
        connections[7] = Connection::new(6, 4);

        // middle
        connections[8] = Connection::new(0, 4);
        connections[9] = Connection::new(1, 5);
        connections[10] = Connection::new(3, 7);
        connections[11] = Connection::new(2, 6);

        // diagonal
        connections[12] = Connection::new(0, 7);
        connections[13] = Connection::new(1, 6);
        connections[14] = Connection::new(2, 5);
        connections[15] = Connection::new(3, 4);

        Body::new(joints.to_vec(), connections.to_vec(), mass)
    }
}

type ClosestPointFn = fn(Vec3, Unit) -> Vec3;

pub struct World {
    bodies: Vec<Body>,
    environment: ClosestPointFn,
    // TPE_ClosestPointFunction environmentFunction;
    // TPE_CollisionCallback collisionCallback;
}

impl World {
    pub fn new(bodies: Vec<Body>, environment: ClosestPointFn) -> Self {
        Self {
            bodies,
            environment,
        }
    }

    pub fn step<F>(&mut self, mut callback: F) -> bool
    where
        F: FnMut(&mut Vec<Body>) -> bool,
    {
        let ret = callback(&mut self.bodies);

        for i in 0..self.bodies.len() {
            let (before, body, after) = split_slice_before_after(&mut self.bodies, i);

            let first_joint_velocity = body.joints.first().map(|j| j.velocity).unwrap_or_default();
            let orig_pos = body.joints.first().unwrap().position;

            // apply joint velocities
            for joint in &mut body.joints {
                // non-rotating bodies will copy the 1st joint's velocity
                if body.flags.contains(BodyFlags::NONROTATING) {
                    joint.velocity = first_joint_velocity
                }

                joint.position += joint.velocity;
            }

            let aabb = body.compute_aabb();

            let mut collided = body.environment_resolve_collision(self.environment);

            if body.flags.contains(BodyFlags::NONROTATING) {
                // Non-rotating bodies may end up still colliding after
                // environment coll resolvement (unlike rotating bodies where
                // each joint is ensured separately to not collide). So if still
                // in collision, we try a few more times. If not successful, we
                // simply undo any shifts we've done. This should absolutely
                // prevent any body escaping out of environment bounds.

                for _ in 0..NONROTATING_COLLISION_RESOLVE_ATTEMPTS {
                    if !collided {
                        break;
                    }

                    collided = body.environment_resolve_collision(self.environment);
                }

                if collided && body.environment_collide(self.environment) {
                    body.move_by(orig_pos - body.joints.first().unwrap().position);
                }
            } else {
                // normal, rotating bodies

                let mut body_tension = Unit::ZERO;

                for connection in &body.connections {
                    let (joint1, joint2) = get_pair_mut(
                        &mut body.joints,
                        connection.joint1 as usize,
                        connection.joint2 as usize,
                    )
                    .unwrap();

                    let dir = joint2.position - joint1.position;
                    let tension = connection_tension(dir.length(), connection.length.get().into());

                    body_tension += tension.abs();

                    if tension.abs() > TENSION_ACCELERATION_THRESHOLD {
                        let mut dir = dir.normalize();

                        if tension.abs() > TENSION_GREATER_ACCELERATION_THRESHOLD {
                            // apply twice the acceleration after a second threshold, not so
                            // elegant but seems to work :)
                            dir.x *= 2;
                            dir.y *= 2;
                            dir.z *= 2;
                        }

                        dir.x /= TENSION_ACCELERATION_DIVIDER;
                        dir.y /= TENSION_ACCELERATION_DIVIDER;
                        dir.z /= TENSION_ACCELERATION_DIVIDER;

                        if tension < 0 {
                            dir.x *= -1;
                            dir.y *= -1;
                            dir.z *= -1;
                        }

                        joint1.velocity += dir;
                        joint2.velocity -= dir;
                    }
                }

                if body.connections.len() > 0 {
                    let hard = !body.flags.contains(BodyFlags::SOFT);

                    if hard {
                        body.reshape(self.environment);

                        let body_tension = body_tension / Unit::from_num(body.connections.len());

                        if body_tension > RESHAPE_TENSION_LIMIT {
                            for _ in 0..RESHAPE_ITERATIONS {
                                body.reshape(self.environment);
                            }
                        }
                    }

                    if !body.flags.contains(BodyFlags::SIMPLE_CONN) {
                        body.cancel_out_velocities(hard);
                    }
                }
            }

            // iterate over first half of bodies only if they are deactivated,
            // iterate over the second half unconditionally
            for other_body in before
                .iter_mut()
                .filter(|b| b.flags.contains(BodyFlags::DEACTIVATED))
                .chain(after.iter_mut())
            {
                // firstly quick-check collision of body AA bounding boxes

                let other_aabb = other_body.compute_aabb();

                if aabb.overlaps_with(&other_aabb)
                    && body.resolve_collision_with_other_body(other_body, self.environment)
                {
                    body.activate();
                    body.deactivate_count = LIGHT_DEACTIVATION;

                    other_body.activate();
                    other_body.deactivate_count = LIGHT_DEACTIVATION;
                }
            }

            if !body.flags.contains(BodyFlags::ALWAYS_ACTIVE) {
                if body.deactivate_count >= DEACTIVATE_AFTER {
                    body.stop();
                    body.deactivate_count = 0;
                    body.flags.insert(BodyFlags::DEACTIVATED);
                } else if body.average_speed() <= LOW_SPEED {
                    body.deactivate_count += 1;
                } else {
                    body.deactivate_count = 0;
                }
            }
        }

        return ret;
    }
}

fn connection_tension(length: Unit, desired_length: Unit) -> Unit {
    length / desired_length
}

/// Panics if slice does not contain any elements.
fn split_slice_before_after<T>(slice: &mut [T], mid: usize) -> (&mut [T], &mut T, &mut [T]) {
    assert!(slice.len() > 0, "cannot split empty slice");

    let (before, after) = slice.split_at_mut(mid);
    let (item, after) = after.split_first_mut().unwrap();

    (before, item, after)
}

pub fn get_pair_mut<'b, T>(
    slice: &'b mut [T],
    first: usize,
    second: usize,
) -> Option<(&'b mut T, &'b mut T)> {
    if first == second {
        return None;
    }

    let first = slice.get_mut(first)? as *mut _;
    let second = slice.get_mut(second)? as *mut _;

    unsafe { Some((&mut *first, &mut *second)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn split_middle_works() {
        let mut arr = [1, 2, 3, 4, 5];

        let result = split_slice_before_after(&mut arr, 1);

        assert_eq!(
            result,
            ([1].as_mut_slice(), &mut 2, [3, 4, 5].as_mut_slice())
        );

        let result = split_slice_before_after(&mut arr, 3);

        assert_eq!(
            result,
            ([1, 2, 3].as_mut_slice(), &mut 4, [5].as_mut_slice())
        );
    }

    #[test]
    fn split_ending_works() {
        let mut arr = [1, 2, 3, 4, 5];

        let result = split_slice_before_after(&mut arr, 4);

        assert_eq!(
            result,
            ([1, 2, 3, 4].as_mut_slice(), &mut 5, [].as_mut_slice())
        );
    }

    #[test]
    fn split_beginning_works() {
        let mut arr = [1, 2, 3, 4, 5];

        let result = split_slice_before_after(&mut arr, 0);

        assert_eq!(
            result,
            ([].as_mut_slice(), &mut 1, [2, 3, 4, 5].as_mut_slice())
        );
    }

    #[test]
    #[should_panic]
    fn zero_slice_first_index() {
        let mut arr: [u8; 0] = [];
        split_slice_before_after(&mut arr, 1);
    }

    #[test]
    #[should_panic]
    fn zero_slice_zero_index() {
        let mut arr: [u8; 0] = [];
        split_slice_before_after(&mut arr, 0);
    }

    #[test]
    #[should_panic]
    fn split_ending_out_of_bounds() {
        let mut arr = [1, 2, 3, 4, 5];

        split_slice_before_after(&mut arr, 5);
    }
}

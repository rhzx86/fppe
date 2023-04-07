use std::{
    num::NonZeroU16,
    ops::{Add, AddAssign},
};

use bitflags::{bitflags, BitFlags};
use fixed::{traits::ToFixed, types::I32F32};
pub use vec::Vec3;

mod vec;

pub type Unit = I32F32;

/// Maximum number of iterations to try to uncollide two colliding bodies.
const COLLISION_RESOLUTION_ITERATIONS: usize = 16;

// Margin by which a body will be shifted back to get out of collision.
const COLLISION_RESOLUTION_MARGIN: Unit =
    Unit::from_bits(0b0000000000000000000000000000000000000100000000000000000000000000); // 1/64

/// Number of times a collision of nonrotating bodies with environment will be
/// attempted to resolve. This probably won't have great performance implications
/// as complex collisions of this kind should be relatively rare.
const NONROTATING_COLLISION_RESOLVE_ATTEMPTS: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct Joint {
    position: Vec3,
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
pub struct BoundingSphere {
    pub center: Vec3,
    pub radius: Unit,
}

/// Physics body made of spheres (each of same weight but possibly different
/// radia) connected by elastic springs.
#[derive(Default, Debug, Hash)]
pub struct Body {
    joints: Vec<Joint>,
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

    pub fn step<F>(&mut self, callback: F) -> bool
    where
        F: Fn(&mut Vec<Body>) -> bool,
    {
        let ret = callback(&mut self.bodies);

        for body in &mut self.bodies {
            let first_joint_velocity = body.joints.first().map(|j| j.velocity).unwrap_or_default();

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
            } else {
                //
            }
        }

        return ret;
    }
}

/// Panics if slice does not contain any elements.
fn split_slice_before_after<T>(slice: &mut [T], mid: usize) -> (&mut [T], &mut T, &mut [T]) {
    assert!(slice.len() > 0, "cannot split empty slice");

    let (before, after) = slice.split_at_mut(mid);
    let (item, after) = after.split_first_mut().unwrap();

    (before, item, after)
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

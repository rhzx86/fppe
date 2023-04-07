/** Completely basic program showing an initialization of a body consisting of
a single joint and dropping it on the floor, then plotting the body vertical
position over time. */
// TPE_Vec3 environmentDistance(TPE_Vec3 point, TPE_Unit maxDistance)
// {
//   return TPE_envGround(point,0); // just an infinite flat plane
// }

// int main(void)
// {
//   TPE_Body body;
//   TPE_World world;
//   TPE_Joint joint;
//   int frame = 0;

//   joint = TPE_joint(TPE_vec3(0,TPE_F * 8,0),TPE_F);
//   TPE_bodyInit(&body,&joint,1,0,0,2 * TPE_F);
//   TPE_worldInit(&world,&body,1,environmentDistance);

//   while (TPE_bodyIsActive(&body))
//   {
//     if (frame % 6 == 0) // print once in 6 frames
//     {
//       TPE_Unit height = TPE_bodyGetCenterOfMass(&body).y;

//       for (int i = 0; i < (height * 4) / TPE_F; ++i)
//         putchar(' ');

//       puts("*");
//     }

//     TPE_bodyApplyGravity(&body,TPE_F / 100);
//     TPE_worldStep(&world);
//     frame++;
//   }

//   puts("body deactivated");
use fixed::traits::Fixed;
//   return 0;
// }
use fppe::*;

fn environment_distance(mut point: Vec3, _max_distance: Unit) -> Vec3 {
    // just an infinite flat plane with ground at 0
    point.y = 0.into();
    point
}

fn main() {
    let joint = Joint::new(Vec3::new(0, 8, 0), 1);
    let body = Body::new(vec![joint], vec![], 2);
    let mut world = World::new(vec![body], environment_distance);
    let mut i = 0;

    let divided_unit = Unit::from_num(1) / Unit::from_num(64);
    println!("0b{:064b}", divided_unit.to_bits());

    loop {
        let ret = world.step(|bodies| {
            let body = bodies.first_mut().unwrap();
            let height = body.center_of_mass().y;

            // println!("height {}", height);

            let downwards_acceleration = Unit::from_num(1) / Unit::from_num(100);

            // println!("{} {:.5}", downwards_acceleration, 1.0 / 100.0);

            for _ in 0..(height * 4).to_num() {
                print!(" ");
            }

            println!("*");

            body.apply_gravity(downwards_acceleration);

            return !body.is_active();
        });

        i += 1;

        if ret || i == 100 {
            break;
        }
    }

    println!("done {}", i);
}

// Completely basic program showing an initialization of a body consisting of
// a single joint and dropping it on the floor, then plotting the body vertical
// position over time.
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

    for i in 0..100 {
        let ret = world.step(|bodies| {
            let body = bodies.first_mut().unwrap();

            if i % 6 == 0 {
                let height = body.center_of_mass().y;

                for _ in 0..(height * 4).to_num() {
                    print!(" ");
                }

                println!("*");
            }

            let downwards_acceleration = Unit::from_num(1) / Unit::from_num(100);
            body.apply_gravity(downwards_acceleration);

            return !body.is_active();
        });

        if ret {
            break;
        }
    }
}

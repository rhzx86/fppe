use console::Term;
use fppe::*;

/** Simple demo showing 2 bodies thrown inside a room, the world is rendered
as a simple ASCII side view. */

const ROOM_SIZE: i32 = 20;

// the following functions are just for ASCII drawing the world

fn env_aa_box_inside(mut point: Vec3, center: Vec3, mut size: Vec3) -> Vec3 {
    size.x = size.x / 2;
    size.y = size.y / 2;
    size.z = size.z / 2;

    let shifted = point - center;

    let mut a = size - shifted;
    let b = shifted + size;

    let mut sx = 1;
    let mut sy = 1;
    let mut sz = 1;

    if b.x < a.x {
        a.x = b.x;
        sx = -1;
    }

    if b.y < a.y {
        a.y = b.y;
        sy = -1;
    }

    if b.z < a.z {
        a.z = b.z;
        sz = -1;
    }

    if a.x < 0 || a.y < 0 || a.z < 0 {
        return point;
    }
    if a.x < a.y {
        if a.x < a.z {
            point.x = center.x + sx * size.x;
        } else {
            point.z = center.z + sz * size.z;
        }
    } else {
        if a.y < a.z {
            point.y = center.y + sy * size.y;
        } else {
            point.z = center.z + sz * size.z;
        }
    }

    return point;
}

fn environment_distance(mut point: Vec3, _max_distance: Unit) -> Vec3 {
    // our environment: just a simple room
    env_aa_box_inside(
        point,
        Vec3::new(
            Unit::from_num(ROOM_SIZE) / 2,
            Unit::from_num(ROOM_SIZE) / 2,
            Unit::ZERO,
        ),
        Vec3::new(ROOM_SIZE, ROOM_SIZE, ROOM_SIZE),
    )
}

const SCREEN_W: usize = 32;
const SCREEN_H: usize = 16;

fn clear_screen(screen: &mut [char; SCREEN_W * SCREEN_H]) {
    for i in 0..(SCREEN_W * SCREEN_H) {
        screen[i] = if i < SCREEN_W || i >= SCREEN_W * (SCREEN_H - 1) {
            '-'
        } else {
            if (i % SCREEN_W) == 0 || (i % SCREEN_W) == SCREEN_W - 1 {
                '|'
            } else {
                ' '
            }
        };
    }
}

fn set_pixel(screen: &mut [char; SCREEN_W * SCREEN_H], x: i32, y: i32, c: char) {
    if x < 0 || x >= (SCREEN_W as i32) || y < 0 || y >= (SCREEN_H as i32) {
        return;
    }

    let y = SCREEN_H - 1 - y as usize;

    screen[y * SCREEN_W + x as usize] = c;
}

fn set_pixel_unit(screen: &mut [char; SCREEN_W * SCREEN_H], x: Unit, y: Unit, c: char) {
    set_pixel(
        screen,
        ((x * Unit::from_num(SCREEN_W)) / Unit::from_num(ROOM_SIZE)).to_num(),
        ((y * Unit::from_num(SCREEN_H)) / Unit::from_num(ROOM_SIZE)).to_num(),
        c,
    )
}

fn print_screen(screen: &[char; SCREEN_W * SCREEN_H]) {
    for _ in 0..20 {
        print!("\n");
    }

    for y in 0..SCREEN_H {
        for x in 0..SCREEN_W {
            print!("{}", screen[y * SCREEN_W + x]);
        }

        print!("\n")
    }
}

fn main() {
    let term = Term::stdout();

    let mut screen: [char; SCREEN_W * SCREEN_H] = ['\0'; SCREEN_W * SCREEN_H];

    let mut first_body = {
        // we'll create the first body "by hand", just two joints (spheres) with one
        // connection:
        let joints = vec![
            Joint::new(
                Vec3::new(
                    3 * Unit::from_num(ROOM_SIZE) / 4,
                    Unit::from_num(ROOM_SIZE) / 2,
                    Unit::from_num(0),
                ),
                1,
            ),
            Joint::new(
                Vec3::new(
                    3 * Unit::from_num(ROOM_SIZE) / 4 + Unit::from_num(1) * 4,
                    Unit::from_num(ROOM_SIZE) / 2,
                    Unit::from_num(0),
                ),
                1,
            ),
        ];

        let connections = vec![Connection::new(0, 1)];

        Body::new(joints, connections, 1)
    };

    // the other (a "box" approximated by spheres) will be made by the library
    // function:
    let mut second_body = Body::make_box(2, 2, 2, 1, 1);
    second_body.move_to(Vec3::new(
        Unit::from_num(ROOM_SIZE) / 2,
        Unit::from_num(ROOM_SIZE) / 2,
        Unit::ZERO,
    ));

    // give some initial velocities and spins to the bodies:

    first_body.accelerate(Vec3::new(
        -1 * Unit::from_num(1) / 8,
        Unit::from_num(1) / 3,
        Unit::from_num(0),
    ));

    first_body.spin(Vec3::new(
        Unit::from_num(0),
        Unit::from_num(0),
        -1 * Unit::from_num(1) / 25,
    ));

    second_body.accelerate(Vec3::new(
        -1 * Unit::from(1) / Unit::from_num(2),
        Unit::ONE / Unit::from_num(50),
        Unit::from_num(0),
    ));

    second_body.spin(Vec3::new(
        Unit::from_num(0),
        Unit::from_num(0),
        Unit::from_num(1) / 23,
    ));

    let mut world = World::new(vec![first_body, second_body], environment_distance);

    // simulate 300 frames
    for i in 0..=300 {
        world.step(|bodies| {
            // draw the world every 10 frames
            if i % 10 == 0 {
                term.clear_screen().unwrap();
                clear_screen(&mut screen);

                for (i, body) in bodies.iter_mut().enumerate() {
                    for joint in &mut body.joints {
                        set_pixel_unit(&mut screen, joint.position.x, joint.position.y, '.');
                    }

                    let com = body.center_of_mass();
                    set_pixel_unit(&mut screen, com.x, com.y, ('A' as u8 + i as u8) as char);
                }

                print_screen(&screen);
                println!("frame {}", i);
                println!("press enter to simulate another frame");
                let _ = term.read_line();
            }

            for body in bodies.iter_mut() {
                body.apply_gravity(Unit::from_num(1) / Unit::from_num(100));
            }

            true
        });
    }
}

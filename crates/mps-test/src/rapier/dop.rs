#[cfg(test)]
mod tests {
    use smallvec::SmallVec;
    use rapier3d::prelude::Vector;
    use mps_core::rapier::dop::*;
    use mps_core::rapier::ffi::*;

    fn cube_points() -> SmallVec<[Vector; 8]> {
        let mut points = SmallVec::new();
        for x in [-1.0, 1.0] {
            for y in [-1.0, 1.0] {
                for z in [-1.0, 1.0] {
                    points.push(Vector::new(x, y, z));
                }
            }
        }
        points
    }

    #[test]
    fn kdop_builds_from_cube_points() {
        let hull = KdopHull {
            directions: kdop_directions(KdopPreset::K14),
        };
        assert!(hull.build(&cube_points()).is_some());
    }

    #[test]
    fn fdh_builds_from_custom_directions() {
        let directions = kdop_directions(KdopPreset::K6);
        let hull = FdhHull {
            directions: &directions,
        };
        assert!(hull.build(&cube_points()).is_some());
    }
}








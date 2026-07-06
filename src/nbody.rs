use nalgebra::Vector3;

const MIN_HALF: f64 = 1e-4;

pub fn brute_force(bodies: &[(Vector3<f64>, f64)], g: f64, soft2: f64) -> Vec<Vector3<f64>> {
    let n = bodies.len();
    let mut forces = vec![Vector3::zeros(); n];
    for i in 0..n {
        let (pi, mi) = bodies[i];
        for j in (i + 1)..n {
            let (pj, mj) = bodies[j];
            let d = pj - pi;
            let r2 = d.norm_squared() + soft2;
            let inv_r3 = 1.0 / (r2 * r2.sqrt());
            let force = d * (g * mi * mj * inv_r3);
            forces[i] += force;
            forces[j] -= force;
        }
    }
    forces
}

pub fn barnes_hut(
    bodies: &[(Vector3<f64>, f64)],
    g: f64,
    soft2: f64,
    theta: f64,
) -> Vec<Vector3<f64>> {
    if bodies.is_empty() {
        return Vec::new();
    }

    let mut min = bodies[0].0;
    let mut max = bodies[0].0;
    for &(p, _) in &bodies[1..] {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        min.z = min.z.min(p.z);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
        max.z = max.z.max(p.z);
    }
    let center = (min + max) * 0.5;
    let extent = max - min;
    let half = extent.x.max(extent.y).max(extent.z) * 0.5 + 1e-3;

    let mut tree = Octree { nodes: Vec::new() };
    let root = tree.push_node(center, half);
    for i in 0..bodies.len() {
        tree.insert(root, i, bodies);
    }

    let theta2 = theta * theta;
    bodies
        .iter()
        .map(|&(p, m)| tree.force_on(p, m, root, g, soft2, theta2))
        .collect()
}

struct Node {
    center: Vector3<f64>,
    half: f64,
    mass: f64,
    com_accum: Vector3<f64>,
    body: Option<usize>,
    children: Option<[usize; 8]>,
}

struct Octree {
    nodes: Vec<Node>,
}

impl Octree {
    fn push_node(&mut self, center: Vector3<f64>, half: f64) -> usize {
        self.nodes.push(Node {
            center,
            half,
            mass: 0.0,
            com_accum: Vector3::zeros(),
            body: None,
            children: None,
        });
        self.nodes.len() - 1
    }

    fn insert(&mut self, ni: usize, bi: usize, bodies: &[(Vector3<f64>, f64)]) {
        let (pos, mass) = bodies[bi];
        self.nodes[ni].mass += mass;
        self.nodes[ni].com_accum += pos * mass;

        if let Some(children) = self.nodes[ni].children {
            let oct = octant(self.nodes[ni].center, pos);
            self.insert(children[oct], bi, bodies);
            return;
        }

        match self.nodes[ni].body.take() {
            None => self.nodes[ni].body = Some(bi),
            Some(existing) => {
                if self.nodes[ni].half < MIN_HALF {
                    self.nodes[ni].body = Some(existing);
                    return;
                }
                let children = self.make_children(ni);
                self.nodes[ni].children = Some(children);
                let epos = bodies[existing].0;
                self.insert(
                    children[octant(self.nodes[ni].center, epos)],
                    existing,
                    bodies,
                );
                self.insert(children[octant(self.nodes[ni].center, pos)], bi, bodies);
            }
        }
    }

    fn make_children(&mut self, ni: usize) -> [usize; 8] {
        let center = self.nodes[ni].center;
        let half = self.nodes[ni].half;
        std::array::from_fn(|oct| self.push_node(child_center(center, half, oct), half * 0.5))
    }

    fn force_on(
        &self,
        pos: Vector3<f64>,
        mass: f64,
        ni: usize,
        g: f64,
        soft2: f64,
        theta2: f64,
    ) -> Vector3<f64> {
        let node = &self.nodes[ni];
        if node.mass == 0.0 {
            return Vector3::zeros();
        }
        let com = node.com_accum / node.mass;
        let d = com - pos;
        let r2 = d.norm_squared();
        let size = node.half * 2.0;

        match node.children {
            Some(children) if size * size >= theta2 * r2 => children
                .iter()
                .map(|&c| self.force_on(pos, mass, c, g, soft2, theta2))
                .sum(),
            _ => {
                let r2s = r2 + soft2;
                d * (g * mass * node.mass / (r2s * r2s.sqrt()))
            }
        }
    }
}

fn octant(center: Vector3<f64>, pos: Vector3<f64>) -> usize {
    (pos.x > center.x) as usize
        | ((pos.y > center.y) as usize) << 1
        | ((pos.z > center.z) as usize) << 2
}

fn child_center(center: Vector3<f64>, half: f64, oct: usize) -> Vector3<f64> {
    let h = half * 0.5;
    Vector3::new(
        center.x + if oct & 1 != 0 { h } else { -h },
        center.y + if oct & 2 != 0 { h } else { -h },
        center.z + if oct & 4 != 0 { h } else { -h },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    const G: f64 = 1.0;
    const SOFT2: f64 = 0.25;

    fn sample_bodies() -> Vec<(Vector3<f64>, f64)> {
        vec![
            (Vector3::new(0.0, 0.0, 0.0), 10.0),
            (Vector3::new(1.5, 0.2, -0.3), 3.0),
            (Vector3::new(-2.0, 1.0, 0.5), 5.0),
            (Vector3::new(3.0, -1.5, 2.0), 1.0),
            (Vector3::new(-1.0, -2.0, -1.0), 7.0),
            (Vector3::new(4.0, 3.0, -2.5), 2.0),
        ]
    }

    #[test]
    fn barnes_hut_approximates_brute_force() {
        let bodies = sample_bodies();
        let exact = brute_force(&bodies, G, SOFT2);
        let approx = barnes_hut(&bodies, G, SOFT2, 0.3);
        for (fe, fa) in exact.iter().zip(&approx) {
            let scale = fe.norm().max(1e-9);
            let err = (fe - fa).norm() / scale;
            assert!(
                err < 0.05,
                "barnes-hut diverged from brute force: exact={fe:?} approx={fa:?} rel_err={err}"
            );
        }
    }

    #[test]
    fn barnes_hut_forces_roughly_cancel_like_brute_force() {
        let bodies = sample_bodies();
        let brute_total: Vector3<f64> = brute_force(&bodies, G, SOFT2).iter().sum();
        let bh_total: Vector3<f64> = barnes_hut(&bodies, G, SOFT2, 0.5).iter().sum();
        assert!(
            brute_total.norm() < 1e-8,
            "brute force total force should be exactly zero by Newton's third law: {brute_total:?}"
        );
        assert!(
            bh_total.norm() < 1e-3,
            "barnes-hut total force should be close to zero too: {bh_total:?}"
        );
    }

    #[test]
    fn empty_and_single_body_feel_no_force() {
        assert!(barnes_hut(&[], G, SOFT2, 0.5).is_empty());
        assert!(brute_force(&[], G, SOFT2).is_empty());

        let one = vec![(Vector3::new(1.0, 2.0, 3.0), 5.0)];
        let bh = barnes_hut(&one, G, SOFT2, 0.5);
        let bf = brute_force(&one, G, SOFT2);
        assert_eq!(bh.len(), 1);
        assert!(bh[0].norm() < 1e-12, "single body felt a force: {:?}", bh[0]);
        assert!(bf[0].norm() < 1e-12, "single body felt a force: {:?}", bf[0]);
    }
}

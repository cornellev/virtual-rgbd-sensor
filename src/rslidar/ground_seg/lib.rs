use segmentation::{NUM_COLS, NUM_RINGS, RangeGraph};

// Implementation of Efficient Online Segmentation for Sparse 3D Laser Scans
// by Bogoslavskyi and Stachniss

// assumptions:
// 1) assume sensor is mounted roughly horzontally on the robot
// 2) assume the curvature of the ground is low
// 3) assume the robot observes the ground plane at least in some pixels
//    of the lowest row of the range image
fn matrix_of_angles(range_graph : RangeGraph) -> Vec<f32> {
    let mut matrix = [f32; (NUM_COLS * NUM_RINGS)];
    for j in 0..NUM_COLS {
        for i in 0..NUM_RINGS {
            matrix[i * NUM_COLS + j] = if i == 0 {
                0.0
            } else { 
                // there will be one less row than RangeGraph
                // turning each column c of the range image R into a stack of angles
                // \alpha_{r-1,c}^r, where each of these angles represents the angle of
                // inclination of a line connecting two points A and B derived from range 
                // readings R_{r-1,c} and R_{r,c} in neighboring rows r-1, r of the range
                // image
                let prev_node = range_graph.nodes[(i-1) * NUM_COLS + j];
                let node = range_graph.nodes[i * NUM_COLS + j];
                let prev_vert = prev_node.elevation_centideg * 100.0;
                let vert = node.elevation_centideg * 100.0;
                // dz = |R_{r-1,c} * sin(vert_{r-1, c}) - R_{r, c} * sin(vert_{r, c})|
                // dx = |R_{r-1,c} * cos(vert_{r, c} - R_{r, c} * cos(vert_{r, c}))|
                let dz = (prev_node.range * prev_vert.sin() - node.range * vert.sin()).abs();
                let dx = (prev_node.range * prev_vert.cos() - node.range * vert.cos()).abs();
                dz.atan2(dx) // atan2(dz, dx)
            }
        }
    }
    matrix
}

fn label_ground(range_graph : RangeGraph) {
    let matrix = matrix_of_angles(range_graph);
    for j in 1..NUM_COLS {
        let idx = 0 * NUM_COLS + j;
        todo!("if M(0, j) not labelled then LabelGroundBFS(0, j)");
    }
}

fn label_ground_bfs(i: usize, j: usize) {
    todo!("queue stuff")
}
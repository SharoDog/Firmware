use pyo3::prelude::*;

fn atan2(y: f64, x: f64) -> f64 {
    let mut t0;
    let mut t1;
    let mut t3;
    let t4;
    t3 = x.abs();
    t1 = y.abs();
    t0 = f64::max(t3, t1);
    t1 = f64::min(t3, t1);
    t3 = 1.0 / t0;
    t3 = t1 * t3;

    t4 = t3 * t3;
    t0 = -0.013480470;
    t0 = t0 * t4 + 0.057477314;
    t0 = t0 * t4 - 0.121239071;
    t0 = t0 * t4 + 0.195635925;
    t0 = t0 * t4 + 0.332994597;
    t0 = t0 * t4 + 0.999995630;
    t3 = t0 * t3;

    if y.abs() > x.abs() {
        t3 = 1.570796327 - t3
    }
    if x < 0.0 {
        t3 = 3.141592654 - t3
    }
    if y < 0.0 {
        t3 = -t3
    }
    t3
}

fn acos(mut x: f64) -> f64 {
    let negate = if x < 0.0 { 1.0 } else { 0.0 };
    x = x.abs();
    let mut ret = -0.0187293;
    ret *= x;
    ret += 0.0742610;
    ret *= x;
    ret -= 0.2121144;
    ret *= x;
    ret += 1.5707288;
    ret *= (1.0 - x).sqrt();
    ret -= 2.0 * negate * ret;
    negate * 3.14159263 + ret
}

/// Calculate inverse kinematics for a point
fn calc_ik(p: &Vec<f64>) -> Vec<f64> {
    // femur length
    let femur = 10.0;
    // tibia length
    let tibia = 9.0;
    // distance from origin, third side of the triangle
    let d = f64::sqrt(p.iter().map(|a| a * a).sum());
    // ZY femur angle
    let omega = atan2(-p[2], -p[1]);
    // XY target vector angle
    let phi = atan2(p[1], p[0]);
    // XY femur angle
    let alpha = acos((femur * femur + d * d - tibia * tibia) / (2.0 * femur * d));
    // angle between femur and tibia
    let beta = acos((femur * femur + tibia * tibia - d * d) / (2.0 * femur * tibia));
    vec![omega, phi - alpha, 3.14 - beta]
}

/// Expose calc_ik to Python
#[pyfunction]
fn ik(p: Vec<f64>) -> PyResult<Vec<f64>> {
    Ok(calc_ik(&p))
}

/// Calculate inverse kinematics for 4 legs and a path
#[pyfunction]
fn path_ik(path: Vec<Vec<Vec<f64>>>) -> PyResult<Vec<Vec<Vec<f64>>>> {
    Ok(path
        .iter()
        .map(|step| step.iter().map(calc_ik).collect())
        .collect())
}

/// A Python module implemented in Rust.
#[pymodule]
fn rutils(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(ik, m)?)?;
    m.add_function(wrap_pyfunction!(path_ik, m)?)?;
    Ok(())
}

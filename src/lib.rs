//! PID(Proportional, Integral, Derivative) controller/control system
//! 
//! A control system is a system that manages, commands, directs or regulates the behavior of other devices/systems 
//! to achieve a desired result.
//! 
//! A PID controller calculates a corrective value as the difference between a desired setpoint and a measured value 
//! and applies a correction based on proportional, integral, and derivative terms. The corrective value is fed back
//! to the system to alter its behaviour towards a certain goal.


//#![no_std] figure out how to make this use no std features

use std::{ops::{Mul, Div, Add, Sub, Neg}, cmp::{PartialOrd},};



#[derive(Clone, Copy)]
pub struct PID<T>{
    previous_error: T,
    previous_integral: T,
    zero_value: T,
    gain_p: T,
    gain_i: T,
    gain_d: T,
    ///if integral windup prevention is desired, set to reasonable limit. otherwise set to None
    integral_limit: Option<T>,
}

impl<T> PID<T>
    where
        T: Mul<Output = T>
        + Div<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Neg<Output = T>
        + PartialOrd 
        + Copy
{
    pub fn new(zero_value: T, gain_p: T, gain_i: T, gain_d: T, integral_limit: Option<T>) -> Self{
        Self{
            previous_error: zero_value,
            previous_integral: zero_value,
            zero_value,
            gain_p,
            gain_i,
            gain_d,
            integral_limit,
        }
    }

    ///pid algorithm implementation
    /// 
    /// set_point: the desired state of your system
    /// measured_value: the current state of your system
    /// delta_time: the loop rate of your system. can be in seconds, milliseconds, hours, etc. depending on your system.
    /// returns a value that should be fed back to your system to correct it
    pub fn calculate(self: &mut Self, set_point: T, measured_value: T, delta_time: T) -> T{
        match delta_time > self.zero_value{
            true => {
                //error is how far off we are
                let error = set_point - measured_value;
                //integral is how long we have had error. kinda. rework this later...
                let mut integral = (self.previous_integral + error) * delta_time;
                //derivative is how quickly we are approaching the correct value
                let derivative = (error - self.previous_error) / delta_time;

                match self.integral_limit{
                    Some(limit) => {
                        if integral > limit{integral = limit}
                        else if integral < -limit{integral = -limit}
                    },
                    None => {},
                }
                
                self.previous_error = error;
                self.previous_integral = integral;
                
                (error * self.gain_p) + (integral * self.gain_i) + (derivative * self.gain_d)
            },
            false => panic!("delta_time cannot be zero")
        }
    }

    pub fn gain_p(self: &Self) -> T{self.gain_p}
    pub fn set_gain_p(self: &mut Self, value: T){self.gain_p = value}

    pub fn gain_i(self: &Self) -> T{self.gain_i}
    pub fn set_gain_i(self: &mut Self, value: T){self.gain_i = value}

    pub fn gain_d(self: &Self) -> T{self.gain_d}
    pub fn set_gain_d(self: &mut Self, value: T){self.gain_d = value}

    pub fn integral_limit(self: &Self) -> Option<T>{self.integral_limit}
    pub fn set_integral_limit(self: &mut Self, value: T){self.integral_limit = Some(value)}

    //pub fn output(self: &Self) -> Option<T>{self.output}
}





#[test]
fn returns_correct_result_with_f64(){ 
    let mut pid = PID::new(0.0, 100.0, 0.0, 0.0, None);
    let output = pid.calculate(50.0, 0.0, 200.0);
    assert!((output/*pid.output.unwrap()*/ - 5000.0_f64).abs() < 0.001);
}
#[test]
fn returns_correct_result_with_i32(){
    let mut pid = PID::new(0, 100, 0, 0, None);
    let output = pid.calculate(50, 0, 200);
    assert!(output/*pid.output.unwrap()*/ == 5000);
}

#[test]
#[should_panic]
fn panic_when_delta_time_0(){
    let mut pid = PID::new(0, 100, 0, 0, None);
    let _ = pid.calculate(50, 0, 0);
}
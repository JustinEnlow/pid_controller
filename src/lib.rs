//! PID(Proportional, Integral, Derivative) controller/control system
//! 
//! A control system is a system that manages, commands, directs or regulates the behavior of other devices/systems 
//! to achieve a desired result.
//! 
//! A PID controller calculates a corrective value as the difference between a desired setpoint and a measured value 
//! and applies a correction based on proportional, integral, and derivative terms. The corrective value is fed back
//! to the system to alter its behaviour towards a certain goal.


use std::{ops::{Mul, Div, Add, Sub, Neg}, cmp::{PartialOrd},};

#[derive(Clone, Copy)]
pub struct PID<T>{
    //set_point: T,
    previous_error: T,
    previous_integral: T,
    zero_value: T,
    pub gain_p: T,
    pub gain_i: T,
    pub gain_d: T,
    ///if integral windup prevention is desired, set to reasonable limit. otherwise set to None
    pub integral_limit: Option<T>,
    //should pid output be a field? what if the user is interested in having this value available for some other use?
    pub output: Option<T>,
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
    pub fn new(/*set_point: T,*/ zero_value: T, gain_p: T, gain_i: T, gain_d: T, integral_limit: Option<T>) -> Self{
        Self{
            //set_point,
            previous_error: zero_value,
            previous_integral: zero_value,
            zero_value,
            gain_p,
            gain_i,
            gain_d,
            integral_limit,
            output: None,
        }
    }

    ///pid algorithm implementation
    /// 
    /// set_point: the desired state of your system
    /// measured_value: the current state of your system
    /// delta_time: the loop rate of your system. can be in seconds, milliseconds, hours, etc. depending on your system.
    /// returns a value that should be fed back to your system to correct it
    pub fn calculate(self: &mut Self, set_point: T, measured_value: T, delta_time: T) /*-> Result<(), &'static str>*/{
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
            
                let output = (error * self.gain_p) + (integral * self.gain_i) + (derivative * self.gain_d);
            
                self.previous_error = error;
                self.previous_integral = integral;

                self.output = Some(output);
                
                /*Ok(())*/
            },
            false => {
                //return Err("delta_time cannot be zero"); //&'static str
                panic!("delta_time cannot be zero")
            }
        }
    }
}


//example use case. pilot controlling rocket forward velocity
//let user_input = 1.0;
    //let max_velocity = 300.0;
    //let delta_time = 200.0;
//
    //let mut pid = PID::new(0.0, 100.0, 0.0, 0.0, None);
//
    //loop{
    //    let measured_value = current_velocity();
    //    let corrective_value = pid.calculate(
    //        user_input * max_velocity, 
    //        measured_value, 
    //        delta_time
    //    );
    //    engine_input(corrective_value);
    //}



#[test]
fn returns_correct_result_with_f64(){ 
    let mut pid = PID::new(0.0, 100.0, 0.0, 0.0, None);
    pid.calculate(50.0, 0.0, 200.0);
    assert!((pid.output.unwrap() - 5000.0_f64).abs() < 0.001);
}
#[test]
fn returns_correct_result_with_i32(){
    let mut pid = PID::new(0, 100, 0, 0, None);
    pid.calculate(50, 0, 200);
    assert!(pid.output.unwrap() == 5000);
}

#[test]
#[should_panic]
fn panic_when_delta_time_0(){
    let mut pid = PID::new(0, 100, 0, 0, None);
    pid.calculate(50, 0, 0);
}
using Physics_Simulation.Extended;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Physics_Simulation
{
    public class VehiclePhysicsSimulation
    {
        /// <summary>
        /// Initialization of VehiclePhysicsSimulation, put in the values that you want to change.
        /// </summary>
        public VehiclePhysicsSimulation(float airDragCoeff, float frontalArea, float rollingCoeff, MassHolder massHolder, Constants constants = new Constants())
        {
            this.airDragCoeff = airDragCoeff;
            this.frontalArea = frontalArea;
            this.rollingCoeff = rollingCoeff;
            this.massHolder = massHolder;
            this.constants = constants;
        }

        /// <summary>
        /// Power in watts
        /// </summary>
        public float Power { get; private set; } = 0;

        /// <summary>
        /// Grade in percent
        /// </summary>
        public float Grade { get; private set; } = 0;

        //Public values
        /// <summary>
        /// Current speed in m/s.
        /// </summary>
        public float SpeedMS { get; private set; } = 0;

        /// <summary>
        /// Current speed in kph.
        /// </summary>
        public float SpeedKPH => SpeedMS * 3.6f;

        /// <summary>
        /// Current speed in mph.
        /// </summary>
        public float SpeedMPH => SpeedMS * 2.237f;

        /// <summary>
        /// Air Drag Coefficient.
        /// </summary>
        public float airDragCoeff = 0.650f;

        /// <summary>
        /// Surface area in m2.
        /// </summary>
        public float frontalArea = 0.375f;

        /// <summary>
        /// Rolling Resistance Coefficient.
        /// </summary>
        public float rollingCoeff = 0.003f;

        /// <summary>
        /// Mass of the vehicle in kg.
        /// </summary>
        public MassHolder massHolder = new MassHolder();

        /// <summary>
        /// Contains constants such as gravity and air density.
        /// </summary>
        public Constants constants = new Constants();

        /// <summary>
        /// Minimum speed before rolling resistance gets activated. Also the smallest speed that can be returned.
        /// </summary>
        public const float minSpeed = 0.01f;

        /// <summary>
        /// Use this to get simulation with constant power. If you want power to vary it's advised to input the power every time in UpdateSpeedPower.
        /// </summary>
        /// <param name="power"></param>
        public void SetConstantPower(float power)
        {
            Power = power;
        }

        /// <summary>
        /// Use this to get simulation with constant grade. If you want the grade to vary it's advised to input the grade every tick in UpdateSpeed.
        /// </summary>
        /// <param name="grade"></param>
        public void SetConstantGrade(float grade)
        {
            Grade = grade;
        }

        /// <summary>
        /// Updates speed at the hand of the passed time, which is passed through as time-step.
        /// </summary>
        /// <param name="timeStep"></param>
        /// <returns></returns>
        public float UpdateSpeed(float timeStep)
        {
            return SpeedMS = GetNewVelocity(Power, SpeedMS, Grade, massHolder.KGs, timeStep);
        }

        /// <summary>
        /// Updates speed at the hand of the passed time, this type also contains power and grade throughput.
        /// </summary>
        /// <param name="timeStep"></param>
        /// <param name="power"></param>
        /// <param name="grade"></param>
        /// <returns></returns>
        public float UpdateSpeed(float timeStep, float power, float grade)
        {
            Power = power;
            Grade = grade;
            return UpdateSpeed(timeStep);
        }

        /// <summary>
        /// Updates speed at the hand of the passed time, this one also takes in power.
        /// </summary>
        /// <param name="timeStep"></param>
        /// <param name="power"></param>
        /// <returns></returns>
        public float UpdateSpeedPower(float timeStep, float power)
        {
            Power = power;
            return UpdateSpeed(timeStep);
        }

        /// <summary>
        /// Updates speed at the hand of the passed time, this one also takes in grade.
        /// </summary>
        /// <param name="timeStep"></param>
        /// <param name="grade"></param>
        /// <returns></returns>
        public float UpdateSpeedGrade(float timeStep, float grade)
        {
            Grade = grade;
            return UpdateSpeed(timeStep);
        }

        private float GetNewVelocity(float power, float velocity, float grade, float mass, float timeStep)
        {
            float powerNeeded = GetTotalForce(velocity, grade, mass) * velocity;
            float netPower = power - powerNeeded;
            float netSpeed = velocity * velocity + 2 * netPower * timeStep / mass;
            float output = (float)Math.Sqrt(Math.Abs(netSpeed));
            return (output < minSpeed) ? 0 : output;
        }

        /// <summary>
        /// Combines FDrag, FRolling, and FGravity.
        /// </summary>
        /// <param name="velocity"></param>
        /// <param name="grade"></param>
        /// <param name="mass"></param>
        /// <returns></returns>
        private float GetTotalForce(float velocity, float grade, float mass) => FDrag(velocity) + FRolling(grade, mass, velocity) + FGravity(grade, mass);

        /// <summary>
        /// Calculates air drag.
        /// </summary>
        /// <param name="velocity"></param>
        /// <returns></returns>
        private float FDrag(float velocity) => 0.5f * airDragCoeff * frontalArea * constants.Rho * velocity * velocity;

        /// <summary>
        /// Calculates rolling resistance.
        /// </summary>
        /// <param name="grade"></param>
        /// <param name="mass"></param>
        /// <param name="velocity"></param>
        /// <returns></returns>
        private float FRolling(float grade, float mass, float velocity)
        {
            //Only apply rolling resistance when the vehicle exceeds a certain speed/is on the move.
            if (!velocity.IsBetween(-minSpeed, minSpeed))
                return constants.G * (float)Math.Cos(Math.Atan(grade / 100)) * mass * rollingCoeff;
            else
                return 0;
        }

        /// <summary>
        /// Calculates gravity resistance/gravity energy release.
        /// </summary>
        /// <param name="grade">In percent. </param>
        /// <param name="mass">In kg. </param>
        /// <returns></returns>
        private float FGravity(float grade, float mass) => constants.G * (float)Math.Sin(Math.Atan(grade / 100)) * mass;
    }

    public struct Constants
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="Rho">Air density in kg/m3, defaults to: 1.225 kg/m3 </param>
        /// <param name="G">Gravitational acceleration in m/s2, defaults to: 9.8067 m/s2 </param>
        public Constants(float Rho = 1.225f, float G = 9.8067f)
        {
            this.Rho = Rho;
            this.G = G;
        }

        /// <summary>
        /// Air density in kg/m3, default: 1.225
        /// </summary>
        public float Rho { get; private set; }

        /// <summary>
        /// Gravitational acceleration in m/s2, default: 9.8067
        /// </summary>
        public float G { get; private set; }
    }

    public struct MassHolder
    {
        public MassHolder(MassType unit, float value)
        {
            KGs = 0; //Compiler screeching avoidant.
            switch (unit)
            {
                case MassType.KGs:
                    KGs = value;
                    break;
                case MassType.Pounds:
                    Pounds = value;
                    break;
            }
        }

        public float Pounds 
        {
            get { return KGs * 2.205f; }
            private set { KGs = value * 0.453592f; }
        }

        public float KGs { get; private set; }

        public enum MassType { KGs, Pounds };
    }
}

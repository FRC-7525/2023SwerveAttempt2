package frc.robot.geometry_helpers.models.Units;

// Storage class to generically store values
public class Length {
    
    double InchToMeterConversion = 0.0254;

    public final double Value;

    public Length(double value){
        Value = value;
    }

    public Length FromInches(double inches){
        return new Length(inches / InchToMeterConversion);
    }

    public Length ToInches(){
        return new Length(Value * InchToMeterConversion);
    }


    //public static double InchesToMeters(){

    //}
}

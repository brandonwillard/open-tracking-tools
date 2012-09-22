package inference;

import inference.InferenceInstance.INFO_LEVEL;

import java.util.Map;

import gov.sandia.cognition.math.matrix.VectorFactory;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingPLFilter;

import com.google.common.collect.Maps;

public class InferenceStore {

	private static final Map<String, InferenceInstance> vehicleToInstance =
		      Maps.newConcurrentMap();

	
	private static VehicleStateInitialParameters defaultVehicleStateInitialParams =
		      new VehicleStateInitialParameters(VectorFactory.getDefault()
		          .createVector2D(150d, 150d), VectorFactory.getDefault()
		          .createVector2D(0.0625, 0.0625), VectorFactory
		          .getDefault().createVector2D(0.000625, 0.000625),
		          VectorFactory.getDefault().createVector2D(0.05d, 1d),
		          VectorFactory.getDefault().createVector2D(1d, 0.05d), 
		          VehicleTrackingPLFilter.class.getName(),
		          50,
		          0l);
	
	
	public static InferenceInstance getInferenceInstance(String vehicleId)
	{
		return vehicleToInstance.get(vehicleId);
	}
	
	
	public static InferenceInstance getOrCreateInferenceInstance(
		    String vehicleId) {

		    InferenceInstance ie = vehicleToInstance.get(vehicleId);

		    if (ie == null) {
		      ie = new InferenceInstance(vehicleId, INFO_LEVEL.SINGLE_RESULT,
		        		  defaultVehicleStateInitialParams);
		      vehicleToInstance.put(vehicleId, ie);
		    }

		    return ie;
	}
	
	public static void removeAll() {
	    
		for(String vehicleId : vehicleToInstance.keySet())
		{
			remove(vehicleId);
		}
	}
	
	public static void remove(String vehicleId) {
		    vehicleToInstance.remove(vehicleId);
		    Observation.remove(vehicleId);
	}
}

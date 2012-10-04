package inference;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicLong;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.ObservationData;
import org.openplans.tools.tracking.impl.VehicleUpdate;
import org.openplans.tools.tracking.impl.VehicleUpdateResponse;

import controllers.Application;
import play.Logger;
import play.Play;
import play.db.jpa.JPA;
import akka.actor.UntypedActor;
import akka.event.Logging;
import akka.event.LoggingAdapter;

public class ObservationActor extends UntypedActor {
	
	public static AtomicLong messageCount = new AtomicLong(0);
	public static AtomicLong maxProcessingTime = new AtomicLong(0);
	public static AtomicLong totalProcessingTime = new AtomicLong(0);
	
	@Override
	public void onReceive(Object update) throws Exception {

		if(update instanceof String)
		{
			Logger.info("message: " , update);
		}
		else if(update instanceof VehicleUpdate && update != null)
		{
			InferenceInstance ie = InferenceStore.getOrCreateInferenceInstance(((VehicleUpdate)update).getVehicleId());
			
			VehicleUpdateResponse response = new VehicleUpdateResponse();
			
			for(ObservationData obsData : ((VehicleUpdate)update).getObservations())
			{
				//Logger.info("Observation received " +  obsData.getVehicleId());
				
				Long startTime = System.nanoTime();
				
				try
				{
					Observation obs = Observation.createObservation(obsData);
				
					ie.update(obs);
				
					response.updateFromVehicleState(ie.getBestState());
				}
				catch(Exception e)
				{
					Logger.info(e.toString());
				}
		       	
				Long endTime = System.nanoTime();
				
				updateStats((endTime - startTime) / 1000000);
			}

			this.getSender().tell(response);
			
		 }
	}
	
	private void updateStats(long miliseconds)
	{
		messageCount.addAndGet(1);
		totalProcessingTime.addAndGet(miliseconds);
		
		synchronized (this)
		{
			if(maxProcessingTime.get() < miliseconds)
				maxProcessingTime.set(miliseconds);
		}
	}

}

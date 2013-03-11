package org.opentrackingtools.util;

import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;


public class TrueObservation extends GpsObservation {

  final private VehicleState<GpsObservation> trueState;

  public TrueObservation(GpsObservation obs, VehicleState<GpsObservation> trueState) {
    super(obs.getSourceId(), obs.getTimestamp(), 
        obs.getObsCoordsLatLon(), obs.getVelocity(), obs.getHeading(), 
        obs.getAccuracy(), obs.getRecordNumber(), obs.getPreviousObservation(), 
        obs.getObsProjected());
    this.trueState = trueState;
  }

  public VehicleState<GpsObservation> getTrueState() {
    return trueState;
  }

}
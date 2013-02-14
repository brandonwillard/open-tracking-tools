package org.opentrackingtools.util;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.impl.SimpleObservation;
import org.opentrackingtools.impl.VehicleState;

public class TrueObservation extends SimpleObservation {

  final private VehicleState trueState;

  public TrueObservation(GpsObservation obs, VehicleState trueState) {
    super(obs.getSourceId(), obs.getTimestamp(), 
        obs.getObsCoordsLatLon(), obs.getVelocity(), obs.getHeading(), 
        obs.getAccuracy(), obs.getRecordNumber(), obs.getPreviousObservation(), 
        obs.getObsProjected());
    this.trueState = trueState;
  }

  public VehicleState getTrueState() {
    return trueState;
  }

}
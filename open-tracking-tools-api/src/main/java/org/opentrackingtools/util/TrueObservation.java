package org.opentrackingtools.util;

import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;

public class TrueObservation extends GpsObservation {

  final private VehicleStateDistribution<GpsObservation> trueState;

  public TrueObservation(GpsObservation obs,
    VehicleStateDistribution<GpsObservation> trueState) {
    super(obs.getSourceId(), obs.getTimestamp(), obs
        .getObsCoordsLatLon(), obs.getVelocity(), obs.getHeading(),
        obs.getAccuracy(), obs.getRecordNumber(), obs
            .getPreviousObservation(), obs.getObsProjected());
    this.trueState = trueState;
  }

  public VehicleStateDistribution<GpsObservation> getTrueState() {
    return this.trueState;
  }

}
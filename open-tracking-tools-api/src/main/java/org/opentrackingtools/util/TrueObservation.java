package org.opentrackingtools.util;

import org.apache.commons.lang3.builder.ToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;

public class TrueObservation extends GpsObservation {

  final private VehicleStateDistribution<GpsObservation> trueState;

  public TrueObservation(GpsObservation obs,
    VehicleStateDistribution<GpsObservation> trueState) {
    super(obs.getSourceId(), obs.getTimestamp(), obs
        .getObsCoordsLatLon(), obs.getVelocity(), obs.getHeading(),
        obs.getFixQuality(), obs.getRecordNumber(), obs
            .getPreviousObservation(), obs.getObsProjected());
    this.trueState = trueState;
  }

  public VehicleStateDistribution<GpsObservation> getTrueState() {
    return this.trueState;
  }

  @Override
  public String toString() {
    final ToStringBuilder builder =
        new ToStringBuilder(this, ToStringStyle.SHORT_PREFIX_STYLE);
    builder.appendSuper(super.toString());
    builder.append("trueState", this.trueState);
    return builder.toString();
  }

}
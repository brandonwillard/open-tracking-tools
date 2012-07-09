package org.openplans.tools.tracking.impl.statistics;

import java.util.Collection;

import org.openplans.tools.tracking.impl.VehicleState;

import gov.sandia.cognition.evaluator.Evaluator;
import gov.sandia.cognition.learning.data.InputOutputPair;
import gov.sandia.cognition.learning.data.TargetEstimatePair;
import gov.sandia.cognition.learning.performance.SupervisedPerformanceEvaluator;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

public class VehicleTrackingPerformanceEvaluator
  extends AbstractCloneableSerializable
    implements
    SupervisedPerformanceEvaluator<VehicleState, VehicleState, VehicleState, 
    VehicleStatePerformanceResult> {

  private static final long serialVersionUID = 2188027766955238277L;

  @Override
  public VehicleStatePerformanceResult evaluatePerformance(
    Evaluator<? super VehicleState, VehicleState> object,
    Collection<? extends InputOutputPair<VehicleState, VehicleState>> data) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public VehicleStatePerformanceResult evaluatePerformance(
    Collection<? extends TargetEstimatePair<VehicleState, VehicleState>> data) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public VehicleTrackingPerformanceEvaluator clone() {
    VehicleTrackingPerformanceEvaluator cloned = (VehicleTrackingPerformanceEvaluator) super.clone();
    return cloned;
  }

}

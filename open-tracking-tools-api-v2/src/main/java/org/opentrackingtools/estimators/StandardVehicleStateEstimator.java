package org.opentrackingtools.estimators;

import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;

import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;

public class StandardVehicleStateEstimator<O extends GpsObservation, V extends VehicleState<O>>
    extends AbstractCloneableSerializable implements
    RecursiveBayesianEstimatorPredictor<O, V> {

  private static final long serialVersionUID = 5185771709018927390L;

  final O observation;
  final V state;

  /**
   * Constructor for creating the initially learned objects, i.e. there is no
   * prior state to work from.
   * 
   * @param observation
   */
  public StandardVehicleStateEstimator(O observation) {
    this.observation = observation;
    this.state = null;
  }

  /**
   * General constructor that specifies the new observation and the
   * mutable/current state off which this estimator works.
   * 
   * @param observation
   * @param state
   */
  public StandardVehicleStateEstimator(O observation, V state) {
    this.observation = observation;
    this.state = state;
  }

  @Override
  public V createInitialLearnedObject() {
    return null;
  }

  @Override
  public void update(V target, Iterable<? extends O> data) {
    // TODO Auto-generated method stub

  }

  @Override
  public void update(V state, O data) {
    // TODO Auto-generated method stub

  }

  @Override
  public V learn(Collection<? extends O> data) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public V createPredictiveDistribution(V posterior) {
    // TODO Auto-generated method stub
    return null;
  }

}

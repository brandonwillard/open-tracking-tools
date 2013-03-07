package org.opentrackingtools.distributions;

import java.util.ArrayList;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;

import gov.sandia.cognition.statistics.DiscreteDistribution;
import gov.sandia.cognition.statistics.ProbabilityMassFunction;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

/**
 * Defines a distribution over paths.  This distribution is discrete
 * 
 * @author bwillard
 *
 * @param <P>
 */
public class PathEdgeDistribution<P extends PathEdge> extends AbstractCloneableSerializable 
  implements DiscreteDistribution<P> {

  private static final long serialVersionUID = -5886297115165936926L;
  
  protected InferenceGraph graph;
  protected VehicleState<? extends GpsObservation> currentState;
  protected Path path;
  

  public PathEdgeDistribution(InferenceGraph graph,
    VehicleState<? extends GpsObservation> currentState, Path path) {
    super();
    this.graph = graph;
    this.currentState = currentState;
    this.path = path;
  }

  public InferenceGraph getGraph() {
    return graph;
  }

  public void setGraph(InferenceGraph graph) {
    this.graph = graph;
  }

  public VehicleState<? extends GpsObservation> getCurrentState() {
    return currentState;
  }

  public void setCurrentState(
    VehicleState<? extends GpsObservation> currentState) {
    this.currentState = currentState;
  }

  public Path getPath() {
    return path;
  }

  public void setPath(Path path) {
    this.path = path;
  }

  @Override
  public P sample(Random random) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public ArrayList<? extends P> sample(Random random, int numSamples) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public ProbabilityMassFunction<P> getProbabilityFunction() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public Set<? extends P> getDomain() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public int getDomainSize() {
    // TODO Auto-generated method stub
    return 0;
  }

}

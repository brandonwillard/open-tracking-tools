package org.opentrackingtools.distributions;

import gov.sandia.cognition.statistics.DiscreteDistribution;
import gov.sandia.cognition.statistics.ProbabilityMassFunction;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.ArrayList;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;

/**
 * Defines a distribution over paths. This distribution is discrete
 * 
 * @author bwillard
 * 
 * @param <P>
 */
public class PathEdgeDistribution<P extends PathEdge> extends
    AbstractCloneableSerializable implements DiscreteDistribution<P> {

  private static final long serialVersionUID = -5886297115165936926L;

  protected VehicleState<? extends GpsObservation> currentState;
  protected InferenceGraph graph;
  protected Path path;

  public PathEdgeDistribution(InferenceGraph graph,
    VehicleState<? extends GpsObservation> currentState, Path path) {
    super();
    this.graph = graph;
    this.currentState = currentState;
    this.path = path;
  }

  public VehicleState<? extends GpsObservation> getCurrentState() {
    return this.currentState;
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

  public InferenceGraph getGraph() {
    return this.graph;
  }

  public Path getPath() {
    return this.path;
  }

  @Override
  public ProbabilityMassFunction<P> getProbabilityFunction() {
    // TODO Auto-generated method stub
    return null;
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

  public void setCurrentState(
    VehicleState<? extends GpsObservation> currentState) {
    this.currentState = currentState;
  }

  public void setGraph(InferenceGraph graph) {
    this.graph = graph;
  }

  public void setPath(Path path) {
    this.path = path;
  }

}

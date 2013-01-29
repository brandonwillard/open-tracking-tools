package org.opentrackingtools.graph.paths.states.impl;

import org.testng.annotations.Test;
import org.testng.annotations.BeforeMethod;
import org.testng.Assert;
import gov.sandia.cognition.statistics.DataDistribution;

import java.util.List;

import org.junit.Ignore;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.impl.Simulation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.Simulation.SimulationParameters;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.VehicleTrackingFilter;

import com.google.common.collect.Lists;

public class SimulationTestHarness {

  private final OtpGraph graph;
  private final Simulation sim;
  private final VehicleTrackingFilter<GpsObservation, VehicleState> filter;
  private final List<DataDistribution<VehicleState>> posteriorDistributions =
      Lists.newArrayList();

  private SimulationTestHarness(String graphName,
    String simName, SimulationParameters simParams,
    VehicleStateInitialParameters infParams,
    VehicleTrackingFilter<GpsObservation, VehicleState> filter) {
    this.graph = new OtpGraph(graphName, null);
    this.sim =
        new Simulation(simName, graph, simParams, infParams);
    this.filter = filter;
  }

  @BeforeMethod(enabled=false)
  public void setUp() throws Exception {
  }

  @Test(enabled=false)
  public void test() {
    VehicleState vehicleState =
        this.sim.computeInitialState();
    long time =
        this.sim.getSimParameters().getStartTime()
            .getTime();
    final DataDistribution<VehicleState> currentState =
        filter.createInitialLearnedObject();

    posteriorDistributions.add(currentState.clone());

    while (time < this.sim.getSimParameters().getEndTime()
        .getTime()) {
      vehicleState = this.sim.stepSimulation(vehicleState);

      time =
          vehicleState.getObservation().getTimestamp()
              .getTime();
      filter.update(currentState,
          vehicleState.getObservation());

      posteriorDistributions.add(currentState.clone());
    }

    Assert.fail("Not yet implemented");
  }

}

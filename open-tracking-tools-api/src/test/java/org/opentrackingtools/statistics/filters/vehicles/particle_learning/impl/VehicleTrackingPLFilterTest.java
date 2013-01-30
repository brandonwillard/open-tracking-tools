package org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl;

import org.testng.annotations.Test;
import org.testng.annotations.BeforeMethod;
import java.lang.reflect.Constructor;

import gov.sandia.cognition.math.matrix.VectorFactory;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.impl.VehicleStateInitialParameters;

public class VehicleTrackingPLFilterTest {
  
    
  @BeforeMethod
  public void setUp() throws Exception {
    /*
     * Create a graph
     */
    
  }
  
  @Test(enabled=false)
  public void f() throws Exception {
    
    VehicleStateInitialParameters vehicleStateInitialParams =
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 30,
            VectorFactory.getDefault().createVector2D(5d,
                95d), VectorFactory.getDefault()
                .createVector2D(95d, 5d),
            VehicleTrackingPLFilter.class.getName(), 25,
            30, 0l);
    
    Class<?> filterType = 
        Class.forName(vehicleStateInitialParams.getFilterTypeName());
      
    Constructor<?> ctor = filterType.getConstructor(GpsObservation.class, 
          InferenceGraph.class,
          VehicleStateInitialParameters.class, 
          Boolean.class);
      
//    VehicleTrackingFilter<GpsObservation, VehicleState> filter = 
//        (VehicleTrackingFilter) ctor.newInstance(initialObs, graph, vehicleStateInitialParams, true);    
    
  }
}

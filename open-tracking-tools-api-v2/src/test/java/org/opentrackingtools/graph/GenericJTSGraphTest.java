package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.Collection;
import java.util.Date;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.apache.commons.collections.CollectionUtils;
import org.apache.commons.collections.Predicate;
import org.apache.commons.collections.Transformer;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.testng.annotations.Test;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class GenericJTSGraphTest {

  @Test
  public void getPaths1() {
    
    final List<LineString> graphEdges = Lists.newArrayList();
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0)}));
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(1, 0), 
        new Coordinate(2, 0)}));
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(1, -1), 
        new Coordinate(1, 0)}));
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(1, 0), 
        new Coordinate(1, 1)}));
    
    final GenericJTSGraph graph = new GenericJTSGraph(graphEdges, false);
    
    final Set<LineString> lineStringsToFind = new HashSet<LineString>();
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0)}));
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0), 
        new Coordinate(2, 0)}));
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0), 
        new Coordinate(1, 1)}));
    
    final InferenceGraphSegment startLine =
        Iterables.getOnlyElement(graph.getNearbyEdges(graphEdges.get(0).getCoordinate(), 0.5d));

    final Coordinate obsCoord = new Coordinate(2, 1);
    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), obsCoord, null, null, null, 0, null,
            new ProjectedCoordinate(null, obsCoord, null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters = new VehicleStateInitialParameters(
        VectorFactory.getDefault().copyArray(new double[] { 0d, 1d, 0d, 0d }), 
        VectorFactory.getDefault().createVector2D(1d, 1d), 0, 
        VectorFactory.getDefault().createVector1D(1e-4d), 0, 
        VectorFactory.getDefault().createVector2D(1e-4d, 1e-4d), 0, 
        VectorFactory.getDefault().createVector2D(1,Double.MAX_VALUE), 
        VectorFactory.getDefault().createVector2D(Double.MAX_VALUE, 1), 
        0, 2, 0);

    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    
    final VehicleStateDistribution<GpsObservation> currentState =
        VehicleStateDistribution.constructInitialVehicleState(parameters, graph, obs, rng, startPathEdge);
    
    Set<Path> paths = graph.getPaths(currentState, obs);
    
    assert(pathsContainLineStrings(paths, lineStringsToFind));
  }
  
  @Test
  public void getPaths2() {
    
    final List<LineString> graphEdges = Lists.newArrayList();
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0)}));
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(1, 0), 
        new Coordinate(2, 0)}));
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(2, 0), 
        new Coordinate(2, 1)}));
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(2, 1), 
        new Coordinate(1, 1)}));
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(1, 0), 
        new Coordinate(1, 1)}));
    graphEdges.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(1, 1), 
        new Coordinate(1, 0)}));
    
    final GenericJTSGraph graph = new GenericJTSGraph(graphEdges, false);
    
    final Set<LineString> lineStringsToFind = new HashSet<LineString>();
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0)}));
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0), 
        new Coordinate(2, 0)}));
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0), 
        new Coordinate(2, 0),
        new Coordinate(2, 1)}));
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0), 
        new Coordinate(2, 0),
        new Coordinate(2, 1),
        new Coordinate(1, 1)}));
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0), 
        new Coordinate(2, 0),
        new Coordinate(2, 1),
        new Coordinate(1, 1),
        new Coordinate(1, 0)}));
    lineStringsToFind.add(JTSFactoryFinder.getGeometryFactory().createLineString(new Coordinate[] { 
        new Coordinate(0, 0), 
        new Coordinate(1, 0), 
        new Coordinate(1, 1)}));
    
    final InferenceGraphSegment startLine =
        Iterables.getOnlyElement(graph.getNearbyEdges(graphEdges.get(0).getCoordinate(), 0.5d));

    final Coordinate obsCoord = new Coordinate(1.5, 0.5);
    final GpsObservation obs =
        new GpsObservation("test", new Date(0l), obsCoord, null, null, null, 0, null,
            new ProjectedCoordinate(null, obsCoord, null));

    final Random rng = new Random(102343292l);

    final VehicleStateInitialParameters parameters = new VehicleStateInitialParameters(
        VectorFactory.getDefault().copyArray(new double[] { 0d, 1d, 0d, 0d }), 
        VectorFactory.getDefault().createVector2D(0.5d, 0.5d), 0, 
        VectorFactory.getDefault().createVector1D(1e-4d), 0, 
        VectorFactory.getDefault().createVector2D(1e-4d, 1e-4d), 0, 
        VectorFactory.getDefault().createVector2D(1,Double.MAX_VALUE), 
        VectorFactory.getDefault().createVector2D(Double.MAX_VALUE, 1), 
        0, 2, 0);

    final PathEdge startPathEdge = new PathEdge(startLine, 0d, false);
    
    final VehicleStateDistribution<GpsObservation> currentState =
        VehicleStateDistribution.constructInitialVehicleState(parameters, graph, obs, rng, startPathEdge);
    
    Set<Path> paths = graph.getPaths(currentState, obs);
    
    assert(pathsContainLineStrings(paths, lineStringsToFind));
  }
  
  private boolean pathsContainLineStrings(Collection<Path> paths, Collection<LineString> lineStrings) {
    
    // Extract the LineString Geometry from each Path into a Collection we can use
    @SuppressWarnings("unchecked")
    final Collection<LineString> pathLineStrings = CollectionUtils.collect(paths, new Transformer() {
      
      @Override
      public Object transform(Object input) {
        Path path = (Path)input;
        if (path.isNullPath())
          return null;
        return path.getGeometry();
      }
    });
    // Remove the null result from the null Path. We don't need it.
    pathLineStrings.remove(null);
    
    // Copy the passed in LineString Set so we can modify it without messing up
    // the data passed in.
    Collection<LineString> lineStringsCopy = new HashSet<LineString>(lineStrings);
    
    // Filter the LineStrings we want to find
    CollectionUtils.filter(lineStringsCopy, new Predicate() {
      @Override
      public boolean evaluate(Object arg0) {
        LineString lineString = (LineString)arg0;
        if (pathLineStrings.contains(lineString)) {
          return false;
        }
        return true;
      }
    });
    
    // If all LineStrings were matched, lineStringsCopy should be empty
    if (lineStringsCopy.isEmpty())
      return true;
    return false;
  }
}

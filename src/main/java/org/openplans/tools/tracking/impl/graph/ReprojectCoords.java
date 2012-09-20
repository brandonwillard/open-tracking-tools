package org.openplans.tools.tracking.impl.graph;


import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.ProjectedCoordinate;
import org.opentripplanner.graph_builder.services.GraphBuilder;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.graph.AbstractVertex;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateFilter;
import com.vividsolutions.jts.geom.Geometry;

/**
 * Reprojects the coordinates of the original graph to a flat coordinate system.
 * 
 * @author novalis
 * 
 */
public class ReprojectCoords implements GraphBuilder {

  @Override
  public void
      buildGraph(Graph graph, HashMap<Class<?>, Object> extra) {
    Field xfield;
    Field yfield;
    Field geomfield;
    try {
      xfield = AbstractVertex.class.getDeclaredField("x");
      xfield.setAccessible(true);
      yfield = AbstractVertex.class.getDeclaredField("y");
      yfield.setAccessible(true);
      geomfield = PlainStreetEdge.class.getDeclaredField("geometry");
      geomfield.setAccessible(true);
    } catch (final SecurityException e) {
      throw new RuntimeException(e);
    } catch (final NoSuchFieldException e) {
      throw new RuntimeException(e);
    }

    // operate on the original graph only
    graph = graph.getService(BaseGraph.class).getBaseGraph();
    graph
        .setVertexComparatorFactory(new SimpleVertexComparatorFactory());
    
    try {
      for (final Vertex v : graph.getVertices()) {
        final AbstractVertex abv = ((AbstractVertex) v);
        final ProjectedCoordinate converted = GeoUtils.convertToEuclidean(
            GeoUtils.reverseCoordinates(abv.getCoordinate()));
        
        xfield.set(abv, converted.x);
        yfield.set(abv, converted.y);
        final ArrayList<Edge> toRemove = new ArrayList<Edge>();
        for (final Edge e : v.getOutgoing()) {
          final Geometry orig = e.getGeometry();
          if (orig == null) {
            toRemove.add(e);
            continue;
          }
          final Geometry geom = GeoUtils.projectLonLatGeom(orig);
          
           // XXX: sets the user data in the geom object to the projection object
          final ProjectedCoordinate tmpConv = GeoUtils.convertToEuclidean(
             GeoUtils.reverseCoordinates(orig.getCoordinate()));
          // TODO FIXME XXX: what about when the geoms cross zones?
          geom.setUserData(new Integer(tmpConv.getUtmZone()));
    
          geomfield.set(e, geom);
        }
        for (final Edge e : toRemove) {
          v.removeOutgoing(e);
          e.getToVertex().removeIncoming(e);
        }
      }
    } catch (final IllegalArgumentException e) {
      throw new RuntimeException(e);
    } catch (final IllegalAccessException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public void checkInputs() {
    // nothing to do
  }

  @Override
  public List<String> getPrerequisites() {
    return Collections.emptyList();
  }

  @Override
  public List<String> provides() {
    return Arrays.asList("reprojected");
  }

}

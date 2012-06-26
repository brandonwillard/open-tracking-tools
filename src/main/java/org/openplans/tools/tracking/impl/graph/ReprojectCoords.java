package org.openplans.tools.tracking.impl.graph;

import static org.openplans.tools.tracking.impl.util.GeoUtils.getCRSTransform;

import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.geotools.geometry.jts.JTS;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentripplanner.graph_builder.services.GraphBuilder;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.graph.AbstractVertex;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

/**
 * Reprojects the coordinates of the original graph to a flat coordinate system.
 * 
 * @author novalis
 * 
 */
public class ReprojectCoords implements GraphBuilder {

  @Override
  public void buildGraph(Graph graph, HashMap<Class<?>, Object> extra) {
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
    } catch (SecurityException e) {
      throw new RuntimeException(e);
    } catch (NoSuchFieldException e) {
      throw new RuntimeException(e);
    }

    // operate on the original graph only
    graph = graph.getService(BaseGraph.class).getBaseGraph();
    graph
        .setVertexComparatorFactory(new SimpleVertexComparatorFactory());
    MathTransform transform = getCRSTransform();
    try {
      for (Vertex v : graph.getVertices()) {
        AbstractVertex abv = ((AbstractVertex) v);
        final Coordinate converted = new Coordinate();

        // reversed coord
        JTS.transform(abv.getCoordinate(), converted, transform);
        xfield.set(abv, converted.x);
        yfield.set(abv, converted.y);
        for (Edge e : v.getOutgoing()) {
          Geometry orig = e.getGeometry();
          Geometry geom = JTS.transform(orig, transform);
          geomfield.set(e, geom);
        }
      }
    } catch (final NoninvertibleTransformException e) {
      throw new RuntimeException(e);
    } catch (final TransformException e) {
      throw new RuntimeException(e);
    } catch (IllegalArgumentException e) {
      throw new RuntimeException(e);
    } catch (IllegalAccessException e) {
      throw new RuntimeException(e);
    }
  }

  @Override
  public List<String> provides() {
    return Arrays.asList("reprojected");
  }

  @Override
  public List<String> getPrerequisites() {
    return Collections.emptyList();
  }

  @Override
  public void checkInputs() {
    // nothing to do
  }

}

package org.opentrackingtools.graph.otp;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.CRS;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchIdentifierException;
import org.opengis.referencing.crs.CRSAuthorityFactory;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.crs.GeographicCRS;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.util.GeoUtils;
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
        final Coordinate vertexCoord = abv.getCoordinate();

        xfield.set(abv, vertexCoord.x);
        yfield.set(abv, vertexCoord.y);
        /*
         * Remove null geometry edges.
         */
        final ArrayList<Edge> toRemove = new ArrayList<Edge>();
        for (final Edge e : v.getOutgoing()) {
          final Geometry orig = e.getGeometry();
          if (orig == null) {
            toRemove.add(e);
            continue;
          }
          final Geometry geom = 
            JTS.transform(orig, getTransform(vertexCoord));

          /*
           * FYI: the user data of the projected geom has the original geom.
           */
          geom.setUserData(orig);

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
    } catch (TransformException e) {
      throw new RuntimeException(e);
    }
  }
  
  private MathTransform getTransform(Coordinate refLonLat) {

    try {
      final CRSAuthorityFactory crsAuthorityFactory =
          CRS.getAuthorityFactory(true);

      final GeographicCRS geoCRS =
          crsAuthorityFactory.createGeographicCRS("EPSG:4326");

      int epsg_code = 32600;
      // add 100 for all zones in southern hemisphere
      if (refLonLat.y < 0) {
        epsg_code += 100;
      }
      // finally, add zone number to code
      epsg_code += GeoUtils.getUTMZoneForLongitude(refLonLat.x);

      final CoordinateReferenceSystem dataCRS =
          crsAuthorityFactory.createCoordinateReferenceSystem("EPSG:"
              + epsg_code);

      final MathTransform transform =
          CRS.findMathTransform(geoCRS, dataCRS);
      return transform;
    } catch (final NoSuchIdentifierException e) {
      e.printStackTrace();
    } catch (final FactoryException e) {
      e.printStackTrace();
    }

    return null;
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

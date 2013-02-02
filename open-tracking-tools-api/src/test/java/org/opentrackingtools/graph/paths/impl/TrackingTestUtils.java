package org.opentrackingtools.graph.paths.impl;

import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import org.geotools.data.shapefile.shp.JTSUtilities;
import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.factory.FactoryRegistryException;
import org.geotools.feature.FeatureIterator;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.geometry.jts.ReferencedEnvelope;
import org.geotools.grid.Lines;
import org.geotools.grid.ortholine.LineOrientation;
import org.geotools.grid.ortholine.OrthoLineDef;
import org.geotools.referencing.CRS;
import org.opengis.feature.Feature;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.util.GeoUtils;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.edgetype.StreetTraversalPermission;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.vertextype.IntersectionVertex;
import org.opentripplanner.routing.vertextype.StreetVertex;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterators;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.algorithm.RobustLineIntersector;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.geomgraph.GeometryGraph;
import com.vividsolutions.jts.geomgraph.index.SimpleMCSweepLineIntersector;
import com.vividsolutions.jts.operation.overlay.EdgeSetNoder;

public class TrackingTestUtils {

  public static LineString makeGeometry(StreetVertex v0,
    StreetVertex v1) {
    final GeometryFactory gf = new GeometryFactory();
    final Coordinate[] coordinates =
        new Coordinate[] { v0.getCoordinate(),
            v1.getCoordinate() };
    return gf.createLineString(coordinates);
  }

  public static SimpleInferredPath makeTmpPath(OtpGraph graph,
    boolean isBackward, Coordinate... coords) {
    Preconditions.checkArgument(coords.length > 1);

    final List<SimpleInferredEdge> edges = Lists.newArrayList();
    StreetVertex vtLast = null;
    for (int i = 0; i < coords.length; i++) {
      final Coordinate coord = coords[i];
      final StreetVertex vt =
          new IntersectionVertex(graph.getBaseGraph(),
              "tmpVertex" + i, coord, "none");
      if (vtLast != null) {
        final double distance =
            vt.getCoordinate().distance(
                vtLast.getCoordinate());
        final LineString geom =
            TrackingTestUtils.makeGeometry(vtLast, vt);
        final Edge edge =
            new PlainStreetEdge(vtLast,
                vt,
                geom,
                //            isBackward ? vt : vtLast, 
                //            isBackward ? vtLast : vt, 
                //            isBackward ? (LineString) geom.reverse() : geom,
                "tmpEdge" + i, distance,
                StreetTraversalPermission.ALL, false);
        final SimpleInferredEdge ie =
            SimpleInferredEdge.getInferredEdge(edge.getGeometry(), edge, 1000 + i, graph);
        edges.add(ie);
      }
      vtLast = vt;

    }

    if (isBackward) {
      Collections.reverse(edges);
    }

    final List<SimplePathEdge> pathEdges = Lists.newArrayList();
    double distToStart = 0;
    for (final SimpleInferredEdge edge : edges) {
      final SimplePathEdge pe =
          SimplePathEdge.getEdge(edge, (isBackward ? -1d : 1d)
              * distToStart, isBackward);
      distToStart += edge.getLength();
      pathEdges.add(pe);
    }

    return SimpleInferredPath.getInferredPath(pathEdges,
        isBackward);
  }

  public static List<LineString> createGridGraph(Coordinate startCoord) throws IOException, NoSuchAuthorityCodeException, FactoryRegistryException, FactoryException {
    List<LineString> edges = Lists.newArrayList();
    
    /*
     * Create a grid
     */
    CoordinateReferenceSystem crs = 
        CRS.getAuthorityFactory(false).createGeographicCRS("EPSG:4326");
    
    Envelope tmpEnv = new Envelope(startCoord);
    
    final double appMeter = GeoUtils.getMetersInAngleDegrees(1d);
    tmpEnv.expandBy(appMeter * 10e3);
    
    ReferencedEnvelope gridBounds = new ReferencedEnvelope(
       tmpEnv.getMinX() ,tmpEnv.getMaxX() , 
       tmpEnv.getMinY() , tmpEnv.getMaxY(), crs);
    
    List<OrthoLineDef> lineDefs = Arrays.asList(
        // vertical (longitude) lines
        new OrthoLineDef(LineOrientation.VERTICAL, 1, appMeter * 100d),
        // horizontal (latitude) lines
        new OrthoLineDef(LineOrientation.HORIZONTAL, 1, appMeter * 100d)
        );

    SimpleFeatureSource grid = Lines.createOrthoLines(gridBounds, lineDefs);
    FeatureIterator iter = grid.getFeatures().features();
   
    List<Object> geoms = Lists.newArrayList();
    while (iter.hasNext()) {
      Feature feature = iter.next();
      LineString geom = (LineString)feature.getDefaultGeometryProperty().getValue();
      /*
       * XXX
       * Using Object so that the correct equals (equalsExact) will be called?
       */
      geoms.add((Object)geom);
    }
    
    Geometry superGeom  = JTSFactoryFinder.getGeometryFactory().buildGeometry(geoms).union();
    
    for (int i = 0; i < ((MultiLineString)superGeom).getNumGeometries(); i++) {
      if (superGeom.getGeometryN(i) instanceof LineString) {
        LineString geom = (LineString) superGeom.getGeometryN(i);
        if (geom.getLength() > 1e-5) {
          edges.add(geom);
          edges.add((LineString)geom.reverse());
        }
      }
    }
    
    return edges;
  }

}

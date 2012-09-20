package org.openplans.tools.tracking.impl.util;

import java.awt.geom.Point2D;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.CRS;
import org.geotools.referencing.operation.projection.PointOutsideEnvelopeException;
import org.opengis.referencing.crs.CRSAuthorityFactory;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateFilter;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.Point;

import com.jhlabs.map.proj.Projection;
import com.jhlabs.map.proj.ProjectionFactory;


public class GeoUtils {
  
  /**
   * From http://gis.stackexchange.com/questions/28986/geotoolkit-conversion-from-lat-long-to-utm
   */
  public static int getEPSGCodefromUTS(double lat, int zone) {
    // define base EPSG code value of all UTM zones;
    int epsg_code = 32600;
    // add 100 for all zones in southern hemisphere
    if (lat < 0) {
       epsg_code += 100;
    }
    // finally, add zone number to code
    epsg_code += zone;  
    
    return epsg_code;
  }
  
  public static ProjectedCoordinate convertToEuclidean(Coordinate latlon) {

    String[] spec = new String[5];
    final int utmZone = GeoUtils.getUTMZoneForLongitude(latlon.y);
    spec[0] = "+proj=utm";
    spec[1] = "+zone=" + utmZone;
    spec[2] = "+ellps=clrk66";
    spec[3] = "+units=m";
    spec[4] = "+no_defs";
    Projection projection = ProjectionFactory.fromPROJ4Specification(spec);
    Point2D.Double from = new Point2D.Double(latlon.y, latlon.x);
    Point2D.Double to = new Point2D.Double();
    to = projection.transform(from, to);

    return new ProjectedCoordinate(projection, utmZone, to);
  }

  public static ProjectedCoordinate convertToEuclidean(Vector vec) {
    return convertToEuclidean(new Coordinate(vec.getElement(0),
        vec.getElement(1)));
  }

  public static Coordinate convertToLatLon(ProjectedCoordinate xy) {
    Point2D.Double from = new Point2D.Double(xy.x, xy.y);
    Point2D.Double to = new Point2D.Double();
    to = xy.getProjection().inverseTransform(from, to);

    return new Coordinate(to.y, to.x);
  }

  public static Projection getProjection(int zone) {
    String[] spec = new String[5];
    spec[0] = "+proj=utm";
    spec[1] = "+zone=" + zone;
    spec[2] = "+ellps=clrk66";
    spec[3] = "+units=m";
    spec[4] = "+no_defs";
    Projection projection = ProjectionFactory.fromPROJ4Specification(spec);
    return projection;
  }
  
  public static Coordinate convertToLatLon(Point2D.Double point, int zone) {
    return convertToLatLon(new ProjectedCoordinate(getProjection(zone), zone, point));
  }
  
  public static Coordinate convertToLatLon(Vector vec, ProjectedCoordinate projCoord) {
    final Point2D.Double point = new Point2D.Double(vec.getElement(0),
        vec.getElement(1));
    return convertToLatLon(new ProjectedCoordinate(projCoord.getProjection(), 
        projCoord.getUtmZone(), point));
  }
  
  public static Coordinate convertToLatLon(Vector vec, Projection projection, int zone) {
    final Point2D.Double point = new Point2D.Double(vec.getElement(0),
        vec.getElement(1));
    return convertToLatLon(new ProjectedCoordinate(projection, zone, point));
  }
  
  /*
   * Taken from OneBusAway's UTMLibrary class
   */
  public static int getUTMZoneForLongitude(double lon) {

    if (lon < -180 || lon > 180)
      throw new IllegalArgumentException(
          "Coordinates not within UTM zone limits");

    int lonZone = (int) ((lon + 180) / 6);

    if (lonZone == 60)
      lonZone--;
    return lonZone + 1;
  }
  
  public static Object getCoordinates(Vector meanLocation) {
    return new Coordinate(meanLocation.getElement(0),
        meanLocation.getElement(1));
  }

  public static Vector getEuclideanVectorFromLatLon(
    Coordinate coordinate) {
    final Coordinate resCoord = convertToEuclidean(coordinate);
    return VectorFactory.getDefault().createVector2D(resCoord.x,
        resCoord.y);
  }


  public static double getMetersInAngleDegrees(double distance) {
    return distance / (Math.PI / 180d) / 6378137d;
  }

  public static Vector getVector(Coordinate coord) {
    return VectorFactory.getDefault()
        .createVector2D(coord.x, coord.y);
  }

  public static Coordinate makeCoordinate(Vector vec) {
    return new Coordinate(vec.getElement(0), vec.getElement(1));
  }

  public static Coordinate reverseCoordinates(Coordinate startCoord) {
    return new Coordinate(startCoord.y, startCoord.x);
  }
  
  /**
   * Finds a UTM projection and applies it to all coordinates of the
   * given geom.
   * @param orig
   * @return
   */

  public static Geometry projectLonLatGeom(Geometry orig) {
    // TODO FIXME XXX: what about when the geoms cross zones?
    final Geometry geom = (Geometry)orig.clone(); 
    geom.apply(new CoordinateFilter() {
      @Override
      public void filter(Coordinate coord) {
        final ProjectedCoordinate converted = GeoUtils.convertToEuclidean(
           GeoUtils.reverseCoordinates(coord));
        coord.setCoordinate(converted);
      }
    });
    
    geom.geometryChanged();
    return geom;
  }
  
  /**
   * Inverts a geometry's projection.
   * @param orig
   * @param projection
   * @return
   */
  public static Geometry invertGeom(Geometry orig, final Projection projection) {
    // TODO FIXME XXX: what about when the geoms cross zones?
    final Geometry geom = (Geometry)orig.clone(); 
    geom.apply(new CoordinateFilter() {
      @Override
      public void filter(Coordinate coord) {
        final Point2D.Double from = new Point2D.Double(coord.x, coord.y); 
        Point2D.Double to = new Point2D.Double(); 
        to = projection.inverseTransform(from, to);
        coord.setCoordinate(new Coordinate(to.x, to.y));
      }
    });
    geom.geometryChanged();
    return geom;
  }
}

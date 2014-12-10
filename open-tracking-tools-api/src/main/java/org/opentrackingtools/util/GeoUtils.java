package org.opentrackingtools.util;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.List;

import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.CRS;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.NoSuchIdentifierException;
import org.opengis.referencing.crs.CRSAuthorityFactory;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.crs.GeographicCRS;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.model.ProjectedCoordinate;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateFilter;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;

public class GeoUtils {

  public static ProjectedCoordinate convertToEuclidean(
    Coordinate latlon) {
    final MathTransform transform = GeoUtils.getTransform(latlon);
    final Coordinate to = new Coordinate();
    try {
      JTS.transform(latlon, to, transform);
    } catch (final TransformException e) {
      e.printStackTrace();
    }

    return new ProjectedCoordinate(transform, to, latlon);
  }

  public static ProjectedCoordinate convertToEuclidean(Vector vec) {
    return GeoUtils.convertToEuclidean(new Coordinate(vec
        .getElement(0), vec.getElement(1)));
  }

  public static Coordinate convertToLatLon(MathTransform transform,
    Coordinate xy) throws NoninvertibleTransformException,
      TransformException {
    final Coordinate to = new Coordinate();
    JTS.transform(xy, to, transform.inverse());
    return new Coordinate(to.x, to.y);
  }

  public static Coordinate convertToLatLon(Vector vec,
    MathTransform transform) throws NoninvertibleTransformException,
      TransformException {
    final Coordinate point =
        new Coordinate(vec.getElement(0), vec.getElement(1));
    return GeoUtils.convertToLatLon(transform, point);
  }

  public static Coordinate convertToLatLon(Vector vec,
    ProjectedCoordinate projCoord)
      throws NoninvertibleTransformException, TransformException {
    final Coordinate point =
        new Coordinate(vec.getElement(0), vec.getElement(1));
    return GeoUtils.convertToLatLon(projCoord.getTransform(), point);
  }

  public static Coordinate getCoordinates(Vector meanLocation) {
    return new Coordinate(meanLocation.getElement(0),
        meanLocation.getElement(1));
  }

  /**
   * From
   * http://gis.stackexchange.com/questions/28986/geotoolkit-conversion-from
   * -lat-long-to-utm
   */
  public static int getEPSGCodefromUTS(Coordinate refLatLon) {
    // define base EPSG code value of all UTM zones;
    int epsg_code = 32600;
    // add 100 for all zones in southern hemisphere
    if (refLatLon.x < 0) {
      epsg_code += 100;
    }
    // finally, add zone number to code
    epsg_code += GeoUtils.getUTMZoneForLongitude(refLatLon.y);

    return epsg_code;
  }

  public static Vector getEuclideanVectorFromLatLon(
    Coordinate coordinate) {
    final Coordinate resCoord =
        GeoUtils.convertToEuclidean(coordinate);
    return VectorFactory.getDefault().createVector2D(resCoord.x,
        resCoord.y);
  }

  public static double getMetersInAngleDegrees(double distance) {
    return distance / (Math.PI / 180d) / 6378137d;
  }

  public static List<LineSegment> getSubLineSegments(
    LineString lineString) {
    Preconditions.checkArgument(lineString.getNumPoints() > 1);
    final List<LineSegment> results = Lists.newArrayList();

    Coordinate prevCoord = lineString.getCoordinateN(0);
    for (int i = 1; i < lineString.getNumPoints(); ++i) {
      final Coordinate nextCoord = lineString.getCoordinateN(i);
      if (!nextCoord.equals2D(prevCoord)) {
        results.add(new LineSegment(prevCoord, nextCoord));
        prevCoord = nextCoord;
      }
    }
    return results;
  }
  
  private static final CRSAuthorityFactory crsAuthorityFactory = CRS.getAuthorityFactory(false);
  private static GeographicCRS geoCRS;
  
  static {
    try {
      geoCRS = crsAuthorityFactory.createGeographicCRS("EPSG:4326");
    } catch (NoSuchAuthorityCodeException e) {
      e.printStackTrace();
    } catch (FactoryException e) {
      e.printStackTrace();
    }
  }
  

  /**
   * Get the geographic CRS that this code assumes.
   * You'll need this to match-up geo CRS between systems, so that 
   * you can afterward properly convert to euclidean, if desired.
   * @return
   */
  public static GeographicCRS getGeographicCRS(){
    return geoCRS;
  }

  /**
   * Get the transform to a local euclidean projection for the
   * area that contains the given reference (lat, lon) coordinate. 
   * 
   * @param refLatLon
   * @return
   */
  public static MathTransform getTransform(Coordinate refLatLon) {

    try {

      final CoordinateReferenceSystem dataCRS =
          crsAuthorityFactory.createCoordinateReferenceSystem("EPSG:"
              + GeoUtils.getEPSGCodefromUTS(refLatLon));

      final MathTransform transform =
          CRS.findMathTransform(geoCRS, dataCRS);
      return transform;
    } catch (final NoSuchIdentifierException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (final FactoryException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    return null;
  }

  /*
   * Taken from OneBusAway's UTMLibrary class
   */
  public static int getUTMZoneForLongitude(double lon) {

    if (lon < -180 || lon > 180) {
      throw new IllegalArgumentException(
          "Coordinates not within UTM zone limits");
    }

    int lonZone = (int) ((lon + 180) / 6);

    if (lonZone == 60) {
      lonZone--;
    }
    return lonZone + 1;
  }

  public static Vector getVector(Coordinate coord) {
    return VectorFactory.getDefault()
        .createVector2D(coord.x, coord.y);
  }

  /**
   * Inverts a geometry's projection.
   * 
   * @param orig
   * @param projection
   * @return
   */
  // TODO FIXME XXX: what about when the geoms cross zones?
  public static Geometry invertGeom(Geometry orig,
    final MathTransform projection) {
    final Geometry geom = (Geometry) orig.clone();
    geom.apply(new CoordinateFilter() {
      @Override
      public void filter(Coordinate coord) {
        final Coordinate to = new Coordinate();
        try {
          JTS.transform(coord, to, projection.inverse());
        } catch (final NoninvertibleTransformException e) {
          e.printStackTrace();
        } catch (final TransformException e) {
          e.printStackTrace();
        }
        coord.setCoordinate(to);
      }
    });
    geom.geometryChanged();
    return geom;
  }

  public static Coordinate makeCoordinate(Vector vec) {
    return new Coordinate(vec.getElement(0), vec.getElement(1));
  }

  /**
   * Finds a UTM projection and applies it to all coordinates of the given geom.
   * 
   * TODO FIXME XXX: what about when the geoms cross zones?
   * 
   * @param orig
   * @return
   */
  public static Geometry projectLonLatGeom(Geometry orig) {
    final Geometry geom = (Geometry) orig.clone();
    geom.apply(new CoordinateFilter() {
      @Override
      public void filter(Coordinate coord) {
        final ProjectedCoordinate converted =
            GeoUtils.convertToEuclidean(coord);
        coord.setCoordinate(converted);
      }
    });

    geom.geometryChanged();
    return geom;
  }
}

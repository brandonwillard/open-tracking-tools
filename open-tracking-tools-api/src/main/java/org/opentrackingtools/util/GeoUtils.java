package org.opentrackingtools.util;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.CRS;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchIdentifierException;
import org.opengis.referencing.crs.CRSAuthorityFactory;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.crs.GeographicCRS;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.util.geom.ProjectedCoordinate;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateFilter;
import com.vividsolutions.jts.geom.Geometry;

public class GeoUtils {

  public static ProjectedCoordinate convertToEuclidean(
    Coordinate latlon) {
    final MathTransform transform = getTransform(latlon);
    final Coordinate to = new Coordinate();
    try {
      JTS.transform(reverseCoordinates(latlon), to,
          transform);
    } catch (final TransformException e) {
      e.printStackTrace();
    }

    return new ProjectedCoordinate(transform, to, latlon);
  }

  public static ProjectedCoordinate convertToEuclidean(
    Vector vec) {
    return convertToEuclidean(new Coordinate(
        vec.getElement(0), vec.getElement(1)));
  }

  public static Coordinate convertToLatLon(
    MathTransform transform, Coordinate xy) {
    final Coordinate to = new Coordinate();
    try {
      JTS.transform(xy, to, transform.inverse());
    } catch (final TransformException e) {
      e.printStackTrace();
    }
    return new Coordinate(to.y, to.x);
  }

  public static Coordinate convertToLatLon(Vector vec,
    MathTransform transform) {
    final Coordinate point =
        new Coordinate(vec.getElement(0), vec.getElement(1));
    return convertToLatLon(transform, point);
  }

  public static Coordinate convertToLatLon(Vector vec,
    ProjectedCoordinate projCoord) {
    final Coordinate point =
        new Coordinate(vec.getElement(0), vec.getElement(1));
    return convertToLatLon(projCoord.getTransform(), point);
  }

  public static Coordinate getCoordinates(
    Vector meanLocation) {
    return new Coordinate(meanLocation.getElement(0),
        meanLocation.getElement(1));
  }

  /**
   * From
   * http://gis.stackexchange.com/questions/28986/geotoolkit-conversion-from
   * -lat-long-to-utm
   */
  public static int
      getEPSGCodefromUTS(Coordinate refLatLon) {
    // define base EPSG code value of all UTM zones;
    int epsg_code = 32600;
    // add 100 for all zones in southern hemisphere
    if (refLatLon.x < 0) {
      epsg_code += 100;
    }
    // finally, add zone number to code
    epsg_code += getUTMZoneForLongitude(refLatLon.y);

    return epsg_code;
  }

  public static Vector getEuclideanVectorFromLatLon(
    Coordinate coordinate) {
    final Coordinate resCoord =
        convertToEuclidean(coordinate);
    return VectorFactory.getDefault().createVector2D(
        resCoord.x, resCoord.y);
  }

  public static double getMetersInAngleDegrees(
    double distance) {
    return distance / (Math.PI / 180d) / 6378137d;
  }

  public static MathTransform getTransform(
    Coordinate refLatLon) {
    //    MathTransformFactory mtFactory = ReferencingFactoryFinder.getMathTransformFactory(null);
    //    ReferencingFactoryContainer factories = new ReferencingFactoryContainer(null);

    final GeographicCRS geoCRS =
        org.geotools.referencing.crs.DefaultGeographicCRS.WGS84;
    try {
      final CRSAuthorityFactory crsAuthorityFactory =
          CRS.getAuthorityFactory(true);

      //      final CoordinateReferenceSystem mapCRS =
      //          crsAuthorityFactory
      //              .createCoordinateReferenceSystem(googleWebMercatorCode);

      final CoordinateReferenceSystem dataCRS =
          crsAuthorityFactory
              .createCoordinateReferenceSystem("EPSG:"
                  + getEPSGCodefromUTS(refLatLon));

      //      parameters = mtFactory.getDefaultParameters("Transverse_Mercator");
      //
      //      final double zoneNumber = zone;
      //      final double utmZoneCenterLongitude = (zoneNumber - 1) * 6 - 180 + 3; // +3 puts origin
      //      parameters.parameter("central_meridian").setValue(utmZoneCenterLongitude);
      //      parameters.parameter("latitude_of_origin").setValue(0.0);
      //      parameters.parameter("scale_factor").setValue(0.9996);
      //      parameters.parameter("false_easting").setValue(500000.0);
      //      parameters.parameter("false_northing").setValue(0.0);
      //  
      //      Map properties = Collections.singletonMap("name", "WGS 84 / UTM Zone " + zoneNumber);
      //      ProjectedCRS projCRS = factories.createProjectedCRS(properties, geoCRS, null, parameters, cartCS);

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
    //    String[] spec = new String[6];
    //    spec[0] = "+proj=utm";
    //    spec[1] = "+zone=" + zone;
    //    spec[2] = "+ellps=clrk66";
    //    spec[3] = "+units=m";
    //    spec[4] = "+datum=NAD83";
    //    spec[5] = "+no_defs";
    //    Projection projection = ProjectionFactory.fromPROJ4Specification(spec);
    //    return projection;
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

  public static Vector getVector(Coordinate coord) {
    return VectorFactory.getDefault().createVector2D(
        coord.x, coord.y);
  }

  /**
   * Inverts a geometry's projection.
   * 
   * @param orig
   * @param projection
   * @return
   */
  public static Geometry invertGeom(Geometry orig,
    final MathTransform projection) {
    // TODO FIXME XXX: what about when the geoms cross zones?
    final Geometry geom = (Geometry) orig.clone();
    geom.apply(new CoordinateFilter() {
      @Override
      public void filter(Coordinate coord) {
        final Coordinate to = new Coordinate();
        try {
          JTS.transform(coord, to, projection.inverse());
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
    return new Coordinate(vec.getElement(0),
        vec.getElement(1));
  }

  /**
   * Finds a UTM projection and applies it to all coordinates of the given geom.
   * 
   * @param orig
   * @return
   */

  public static Geometry projectLonLatGeom(Geometry orig) {
    // TODO FIXME XXX: what about when the geoms cross zones?
    final Geometry geom = (Geometry) orig.clone();
    geom.apply(new CoordinateFilter() {
      @Override
      public void filter(Coordinate coord) {
        final ProjectedCoordinate converted =
            GeoUtils.convertToEuclidean(GeoUtils
                .reverseCoordinates(coord));
        coord.setCoordinate(converted);
      }
    });

    geom.geometryChanged();
    return geom;
  }

  public static Coordinate reverseCoordinates(
    Coordinate startCoord) {
    return new Coordinate(startCoord.y, startCoord.x);
  }
}

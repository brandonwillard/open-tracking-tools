package org.openplans.tools.tracking.impl.util;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.CRS;
import org.opengis.referencing.crs.CRSAuthorityFactory;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;

import com.vividsolutions.jts.geom.Coordinate;

public class GeoUtils {

  public static class GeoSetup {
    
    final CoordinateReferenceSystem mapCRS;
    final CoordinateReferenceSystem dataCRS;
    final MathTransform transform;
    
    public GeoSetup(CoordinateReferenceSystem mapCRS,
      CoordinateReferenceSystem dataCRS, MathTransform transform) {
      this.mapCRS = mapCRS;
      this.dataCRS = dataCRS;
      this.transform = transform;
    }

    public CoordinateReferenceSystem getMapCRS() {
      return mapCRS;
    }

    public CoordinateReferenceSystem getDataCRS() {
      return dataCRS;
    }

    public MathTransform getTransform() {
      return transform;
    }
    
  }
  
  public static ThreadLocal<GeoSetup> geoData = new ThreadLocal<GeoSetup>() {

    @Override
    public GeoSetup get() {
      return super.get();
    }

    @Override
    protected GeoSetup initialValue() {
      System.setProperty("org.geotools.referencing.forceXY", "true");
      try {
        // EPSG:4326 -> WGS84
        // EPSG:3785 is web mercator
        final String googleWebMercatorCode = "EPSG:4326";

        // Projected CRS
        // CRS code: 3785
        final String cartesianCode = "EPSG:4499";

        // UTM zone 51N
        // final String cartesianCode = "EPSG:3829";

        final CRSAuthorityFactory crsAuthorityFactory = CRS
            .getAuthorityFactory(true);

        final CoordinateReferenceSystem mapCRS = crsAuthorityFactory
            .createCoordinateReferenceSystem(googleWebMercatorCode);

        final CoordinateReferenceSystem dataCRS = crsAuthorityFactory
            .createCoordinateReferenceSystem(cartesianCode);

        final boolean lenient = true; // allow for some error due to different
                                      // datums
        final MathTransform transform = CRS.findMathTransform(mapCRS, dataCRS, lenient);
        
        return new GeoSetup(mapCRS, dataCRS, transform);
        
      } catch (final Exception e) {
        e.printStackTrace();
      }

      return null;
    }

  };
  
  public static Coordinate convertToEuclidean(Coordinate latlon) {
    final Coordinate converted = new Coordinate();

    try {
      /*
       * CRS is lon-lat order
       */
      final Coordinate lonlat = reverseCoordinates(latlon);
      JTS.transform(lonlat, converted,
          getCRSTransform());
    } catch (final NoninvertibleTransformException e) {
      e.printStackTrace();
    } catch (final TransformException e) {
      e.printStackTrace();
    }

    return converted;
  }

  public static Coordinate convertToEuclidean(Vector vec) {
    return convertToEuclidean(new Coordinate(vec.getElement(0),
        vec.getElement(1)));
  }

  public static Coordinate convertToLatLon(Coordinate xy) {
    final Coordinate converted = new Coordinate();
    try {
      JTS.transform(xy, converted, getCRSTransform().inverse());
    } catch (final NoninvertibleTransformException e) {
      e.printStackTrace();
    } catch (final TransformException e) {
      e.printStackTrace();
    }

    return new Coordinate(converted.y, converted.x);
  }

  public static Coordinate convertToLatLon(Vector vec) {
    return convertToLatLon(new Coordinate(vec.getElement(0), vec.getElement(1)));
  }

  public static MathTransform getCRSTransform() {
    return geoData.get().getTransform();
  }
  
  public static CoordinateReferenceSystem getMapCRS() {
    return geoData.get().getMapCRS();
  }
  
  public static CoordinateReferenceSystem getDataCRS() {
    return geoData.get().getDataCRS();
  }

  public static Coordinate reverseCoordinates(Coordinate startCoord) {
    return new Coordinate(startCoord.y, startCoord.x);
  }

  public static Vector getEuclideanVector(Coordinate coordinate) {
    Coordinate resCoord = convertToEuclidean(coordinate);
    return VectorFactory.getDefault().createVector2D(resCoord.x, resCoord.y);
  }
  
  public static double getAngleDegreesInMeters(double angularUnits) {
    return angularUnits * (Math.PI/180d) * 6378137d; 
  }

  public static double getMetersInAngleDegrees(double distance) {
    return distance / (Math.PI/180d) / 6378137d; 
  }
}

package org.openplans.tools.tracking.impl.util;

import gov.sandia.cognition.math.matrix.Vector;

import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.CRS;
import org.opengis.referencing.crs.CRSAuthorityFactory;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;

import com.vividsolutions.jts.geom.Coordinate;

public class GeoUtils {

  public static ThreadLocal<MathTransform> transform = new ThreadLocal<MathTransform>() {

    @Override
    public MathTransform get() {
      return super.get();
    }

    @Override
    protected MathTransform initialValue() {
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
        return CRS.findMathTransform(mapCRS, dataCRS, lenient);
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
      JTS.transform(new Coordinate(latlon.y, latlon.x), converted,
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
    return transform.get();
  }

  public static Coordinate getLonLat(Coordinate startCoord) {
    return new Coordinate(startCoord.y, startCoord.x);
  }
}

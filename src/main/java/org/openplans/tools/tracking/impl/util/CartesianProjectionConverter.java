package org.openplans.tools.tracking.impl.util;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;

import org.apache.log4j.Logger;
import org.geotools.geometry.jts.JTS;
import org.geotools.referencing.CRS;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.crs.CRSAuthorityFactory;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;

import au.com.bytecode.opencsv.CSVReader;

import com.vividsolutions.jts.geom.Coordinate;

public class CartesianProjectionConverter {

  private static final org.apache.log4j.Logger log = Logger
      .getLogger(CartesianProjectionConverter.class);

  /**
   * @param args
   */
  public static void main(String[] args) {
    // TODO Auto-generated method stub
    final String googleWebMercatorCode = "EPSG:4326";

    final String cartesianCode = "EPSG:4499";

    final CRSAuthorityFactory crsAuthorityFactory = CRS
        .getAuthorityFactory(true);
    CoordinateReferenceSystem mapCRS;
    CoordinateReferenceSystem dataCRS;
    MathTransform transform;

    final CSVReader gps_reader;
    final FileWriter test_output;
    try {
      mapCRS = crsAuthorityFactory
          .createCoordinateReferenceSystem(googleWebMercatorCode);
      dataCRS = crsAuthorityFactory
          .createCoordinateReferenceSystem(cartesianCode);
      final boolean lenient = true; // allow for some error due to different
                                    // datums
      transform = CRS.findMathTransform(mapCRS, dataCRS, lenient);
      test_output = new FileWriter(
          "src/main/resources/org/openplans/cebutaxi/test_data/proj_output.csv");
      test_output.write("time,proj_x,proj_y\n");

      gps_reader = new CSVReader(
          new FileReader(
              "src/main/resources/org/openplans/cebutaxi/test_data/Cebu-Taxi-GPS/0726.csv"),
          ',');
      String[] nextLine;
      gps_reader.readNext();
      log.info("processing gps data");

      final SimpleDateFormat sdf = new SimpleDateFormat("F/d/y H:m:s");

      while ((nextLine = gps_reader.readNext()) != null) {
        final Date datetime = sdf.parse(nextLine[0]);
        log.info("processing record time " + datetime.toString());

        test_output.write(datetime.getTime() + ",");
        /*
         * Transform gps observation to cartesian coordinates
         */
        final double lat = Double.parseDouble(nextLine[2]);
        final double lon = Double.parseDouble(nextLine[3]);
        final Coordinate obsCoords = new Coordinate(lon, lat);
        final Coordinate obsPoint = new Coordinate();
        JTS.transform(obsCoords, obsPoint, transform);

        test_output.write(obsPoint.x + "," + obsPoint.y + "\n");

      }
    } catch (final IOException e) {
      e.printStackTrace();
    } catch (final TransformException e) {
      e.printStackTrace();
    } catch (final NoSuchAuthorityCodeException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (final FactoryException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (final ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

  }

}

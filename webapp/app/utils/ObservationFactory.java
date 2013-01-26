package utils;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.io.Serializable;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Map;

import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.impl.SimpleObservation;
import org.opentrackingtools.impl.TimeOrderException;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.geom.ProjectedCoordinate;

import com.google.common.collect.ComparisonChain;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;

public class ObservationFactory {

  /*
   * This map is how we keep track of the previous records for each vehicle.
   */
  private static Map<String, SimpleObservation> vehiclesToRecords =
      Maps.newConcurrentMap();

  private static final SimpleDateFormat sdf =
      new SimpleDateFormat("yyyy-MM-dd hh:mm:ss");

  public static void clearRecordData() {
    vehiclesToRecords.clear();
  }

  public static GpsObservation createObservation(
    SimpleObservation obsData) throws TimeOrderException {
    return createObservation(obsData.getSourceId(),
        obsData.getTimestamp(),
        obsData.getObsCoordsLatLon(),
        obsData.getVelocity(), obsData.getHeading(),
        obsData.getAccuracy());
  }

  public static synchronized GpsObservation createObservation(
    String vehicleId, Date time, Coordinate obsCoords,
    Double velocity, Double heading, Double accuracy)
      throws TimeOrderException {
    final ProjectedCoordinate obsPoint =
        GeoUtils.convertToEuclidean(obsCoords);

    final SimpleObservation prevObs =
        vehiclesToRecords.get(vehicleId);

    /*
     * do this so we don't potentially hold on to every record in memory
     */
    final int recordNumber;
    if (prevObs != null) {
      /*
       * We check for out-of-time-order records.
       */
      if (time.getTime() <= prevObs.getTimestamp()
          .getTime())
        throw new TimeOrderException();

      recordNumber = prevObs.getRecordNumber() + 1;
      prevObs.reset();

    } else {
      recordNumber = 0;
    }

    final SimpleObservation obs =
        new SimpleObservation(vehicleId, time, obsCoords,
            velocity, heading, accuracy, recordNumber, 
            prevObs, obsPoint);

    vehiclesToRecords.put(vehicleId, obs);

    return obs;
  }

  public static synchronized GpsObservation createObservation(
    String vehicleId, String timestamp, String latStr,
    String lonStr, String velocity, String heading,
    String accuracy) throws NumberFormatException,
      ParseException, TransformException,
      TimeOrderException {

    final double lat = Double.parseDouble(latStr);
    final double lon = Double.parseDouble(lonStr);
    final Coordinate obsCoords = new Coordinate(lat, lon);

    final Double velocityd =
        velocity != null ? Double.parseDouble(velocity)
            : null;
    final Double headingd =
        heading != null ? Double.parseDouble(heading)
            : null;
    final Double accuracyd =
        accuracy != null ? Double.parseDouble(accuracy)
            : null;
    final Date time = sdf.parse(timestamp);

    return createObservation(vehicleId, time, obsCoords,
        velocityd, headingd, accuracyd);
  }

  public static SimpleDateFormat getSdf() {
    return sdf;
  }

  public static Map<String, SimpleObservation>
      getVehiclesToRecords() {
    return vehiclesToRecords;
  }

  public static void remove(String name) {
    vehiclesToRecords.remove(name);
  }

}

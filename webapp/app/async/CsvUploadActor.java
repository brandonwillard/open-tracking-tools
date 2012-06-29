package async;

import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.math.RingAccumulator;
import inference.InferenceService;

import java.io.File;
import java.io.FileReader;
import java.text.ParseException;

import org.opengis.referencing.operation.TransformException;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.TimeOrderException;

import akka.actor.UntypedActor;
import akka.event.Logging;
import akka.event.LoggingAdapter;
import au.com.bytecode.opencsv.CSVReader;

import com.google.common.base.Stopwatch;

public class CsvUploadActor extends UntypedActor {
  LoggingAdapter log = Logging.getLogger(getContext().system(), this);

  @Override
  public void onReceive(Object csvFile) throws Exception {
    if (csvFile instanceof File) {

      final CSVReader gps_reader = new CSVReader(new FileReader(
          (File) csvFile), ';');

      log.info("processing gps data from "
          + ((File) csvFile).getName());

      // skip header
      gps_reader.readNext();

      String[] line = gps_reader.readNext();

      // clear previous trace for this data
      InferenceService.remove("trace-" + line[3]);
      Observation.remove("trace-" + line[3]);

      final RingAccumulator<MutableDouble> averager = new RingAccumulator<MutableDouble>();
      int i = 0;
      do {
        final Stopwatch watch = new Stopwatch();
        watch.start();
        try {
          traceLocation(
              ((File) csvFile).getName(), "trace-" + line[3],
              line[1], line[5], line[7], line[10], null, null);
          log.info("processed time: " + line[1]);
        } catch (final TimeOrderException ex) {
          log.info("bad time order: " + line);
        } catch (final Exception e) {
          log.error("bad csv line: " + line.toString()
              + "\n Exception:" + e.getMessage()); // bad line
          e.printStackTrace();
          break;
        }
        watch.stop();
        averager.accumulate(new MutableDouble(watch.elapsedMillis()));

        if (i % 20 == 0)
          log.info("avg. secs per record = " + averager.getMean().value
              / 1000d);

        i++;
      } while ((line = gps_reader.readNext()) != null);

      log.info("finished processing " + ((File) csvFile).getName());
    }

  }

  public static void traceLocation(String csvFileName,
    String vehicleId, String timestamp, String latStr, String lonStr,
    String velocity, String heading, String accuracy)
      throws NumberFormatException, ParseException,
      TransformException, TimeOrderException {

    final Observation location = Observation.createObservation(
        vehicleId, timestamp, latStr, lonStr, velocity, heading,
        accuracy);

    // TODO set flags for result record handling
    InferenceService.processRecord(location);

  }
}
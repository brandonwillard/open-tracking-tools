package async;

import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.math.RingAccumulator;
import inference.InferenceService;
import inference.InferenceService.INFO_LEVEL;

import java.io.File;
import java.io.FileReader;
import java.text.ParseException;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import models.InferenceInstance;

import org.opengis.referencing.operation.TransformException;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.TimeOrderException;

import akka.actor.UntypedActor;
import akka.event.Logging;
import akka.event.LoggingAdapter;
import au.com.bytecode.opencsv.CSVReader;

import com.google.common.base.Stopwatch;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

public class CsvUploadActor extends UntypedActor {
  LoggingAdapter log = Logging.getLogger(getContext().system(), this);
  
  public static class TraceParameters {

    private File dest;
    private boolean debugEnabled;

    public TraceParameters(File dest, boolean debugEnabled) {
      this.dest = dest;
      this.debugEnabled = debugEnabled;
    }

    public File getDest() {
      return dest;
    }

    public boolean isDebugEnabled() {
      return debugEnabled;
    }
    
  }

  @Override
  public void onReceive(Object params) throws Exception {
    if (params instanceof TraceParameters) {

      final TraceParameters traceParams = (TraceParameters) params;
      final CSVReader gps_reader = new CSVReader(new FileReader(
          (File) traceParams.getDest()), ';');

      final String filename = ((File) traceParams.getDest()).getName();
      log.info("processing gps data from "
          + filename );

      // skip header
      gps_reader.readNext();

      String[] line = gps_reader.readNext();

//      // clear previous trace for this data
//      InferenceService.remove("trace-" + line[3]);
//      Observation.remove("trace-" + line[3]);

      Set<String> vehicleIds = Sets.newHashSet();
      List<Observation> observations = Lists.newArrayList();
      do {
        try {
          String vehicleId = "trace-" + line[3];
          vehicleIds.add(vehicleId);
          
          final Observation obs = Observation.createObservation(
              vehicleId, line[1], line[5], line[7], line[10], null, null);
          observations.add(obs);
          
        } catch (final TimeOrderException ex) {
          log.info("bad time order: " + com.google.common.base.Joiner.on(", ").join(line));
        } catch (final Exception e) {
          log.error("bad csv line: " + com.google.common.base.Joiner.on(", ").join(line)
              + "\n Exception:" + e.getMessage()); // bad line
          e.printStackTrace();
          break;
        }

      } while ((line = gps_reader.readNext()) != null);

      INFO_LEVEL level = traceParams.isDebugEnabled() ? 
          INFO_LEVEL.DEBUG : InferenceService.defaultInfoLevel;
      
      InferenceService.processRecords(observations, level);
      InferenceService.getExecutor().awaitTermination(5, TimeUnit.SECONDS);
      
      log.info("finished processing " + filename);
      
      for (String vehicleId : vehicleIds) {
        InferenceInstance ie = InferenceService.getInferenceInstance(vehicleId);
        if (ie != null)
          log.info("avg. secs per record = " + ie.getAverager().getMean().value
              / 1000d);
        
      }
    }
  }

}
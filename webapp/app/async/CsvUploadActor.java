package async;

import inference.InferenceService;
import inference.InferenceService.INFO_LEVEL;

import java.io.File;
import java.io.FileReader;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import models.InferenceInstance;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.TimeOrderException;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;

import akka.actor.UntypedActor;
import akka.event.Logging;
import akka.event.LoggingAdapter;
import au.com.bytecode.opencsv.CSVReader;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

public class CsvUploadActor extends UntypedActor {
  public static class TraceParameters {

    private final File dest;
    private final boolean debugEnabled;
    private final VehicleStateInitialParameters vehicleStateParams;
    private final String filterTypeName;

    public TraceParameters(File dest,
      VehicleStateInitialParameters vehicleStateParams, boolean debugEnabled) {
      this.vehicleStateParams = vehicleStateParams;
      this.filterTypeName = vehicleStateParams.getFilterTypeName();
      this.dest = dest;
      this.debugEnabled = debugEnabled;
    }

    public File getDest() {
      return dest;
    }

    public VehicleStateInitialParameters
        getVehicleStateInitialParams() {
      return vehicleStateParams;
    }

    public boolean isDebugEnabled() {
      return debugEnabled;
    }

    public String getFilterTypeName() {
      return filterTypeName;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result = prime * result + (debugEnabled ? 1231 : 1237);
      result =
          prime * result + ((dest == null) ? 0 : dest.hashCode());
      result =
          prime
              * result
              + ((filterTypeName == null) ? 0 : filterTypeName
                  .hashCode());
      result =
          prime
              * result
              + ((vehicleStateParams == null) ? 0
                  : vehicleStateParams.hashCode());
      return result;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (obj == null) {
        return false;
      }
      if (getClass() != obj.getClass()) {
        return false;
      }
      TraceParameters other = (TraceParameters) obj;
      if (debugEnabled != other.debugEnabled) {
        return false;
      }
      if (dest == null) {
        if (other.dest != null) {
          return false;
        }
      } else if (!dest.equals(other.dest)) {
        return false;
      }
      if (filterTypeName == null) {
        if (other.filterTypeName != null) {
          return false;
        }
      } else if (!filterTypeName.equals(other.filterTypeName)) {
        return false;
      }
      if (vehicleStateParams == null) {
        if (other.vehicleStateParams != null) {
          return false;
        }
      } else if (!vehicleStateParams.equals(other.vehicleStateParams)) {
        return false;
      }
      return true;
    }

  }

  LoggingAdapter log = Logging.getLogger(getContext().system(), this);

  @Override
  public void onReceive(Object params) throws Exception {
    if (params instanceof TraceParameters) {

      final TraceParameters traceParams = (TraceParameters) params;
      final CSVReader gps_reader =
          new CSVReader(new FileReader(traceParams.getDest()), ';');

      final String filename = traceParams.getDest().getName();
      log.info("processing gps data from " + filename);

      // skip header
      gps_reader.readNext();

      String[] line = gps_reader.readNext();


      final Set<String> vehicleIds = Sets.newHashSet();
      final List<Observation> observations = Lists.newArrayList();
      try {
        do {
          try {
            final String vehicleId = "trace-" + line[3] + traceParams.hashCode();
            vehicleIds.add(vehicleId);
            
            // clear previous trace for this data
            InferenceService.remove(vehicleId);
            Observation.remove(vehicleId);

            final Observation obs =
                Observation.createObservation(vehicleId, line[6],
                    line[4], line[5], line[7], null, null);
            observations.add(obs);

          } catch (final TimeOrderException ex) {
            log.info("bad time order: "
                + com.google.common.base.Joiner.on(", ").join(line));
          }
        } while ((line = gps_reader.readNext()) != null);
      } catch (final Exception e) {
        log.error("bad csv line: "
            + com.google.common.base.Joiner.on(", ").join(line)
            + "\n Exception:" + e.getMessage()); // bad line
        e.printStackTrace();
        gps_reader.close();
        return;
      }

      gps_reader.close();

      final INFO_LEVEL level =
          traceParams.isDebugEnabled() ? INFO_LEVEL.DEBUG
              : InferenceService.defaultInfoLevel;

      InferenceService.processRecords(observations,
          traceParams.getVehicleStateInitialParams(), traceParams.getFilterTypeName(), level);
      InferenceService.getExecutor().awaitTermination(5,
          TimeUnit.SECONDS);

      log.info("finished processing " + filename);

      for (final String vehicleId : vehicleIds) {
        final InferenceInstance ie =
            InferenceService.getInferenceInstance(vehicleId);
        if (ie != null)
          log.info("avg. records per sec = " + 1000d
              / ie.getAverager().getMean().value);

      }
    }
  }

}
package org.openplans.tools.tracking.server;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import org.apache.commons.fileupload.FileItem;
import org.apache.commons.fileupload.FileItemFactory;
import org.apache.commons.fileupload.FileUploadBase;
import org.apache.commons.fileupload.disk.DiskFileItemFactory;
import org.apache.commons.fileupload.servlet.ServletFileUpload;
import org.apache.commons.fileupload.servlet.ServletRequestContext;
import org.apache.commons.io.IOUtils;
import org.apache.log4j.Logger;
import org.openplans.tools.tracking.client.TraceUploadService;
import org.openplans.tools.tracking.server.InferenceServiceImpl.INFO_LEVEL;
import org.openplans.tools.tracking.server.shared.InferenceInstance;
import org.opentrackingtools.impl.Observation;
import org.opentrackingtools.impl.TimeOrderException;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;

import au.com.bytecode.opencsv.CSVReader;

import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import com.google.gwt.user.server.rpc.RemoteServiceServlet;

/**
 * Servlet implementation class TraceUploadService
 */
public class TraceUploadServiceImpl extends RemoteServiceServlet implements TraceUploadService {
	private static final long serialVersionUID = 1L;
       
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

  public TraceUploadServiceImpl() {
    super();
  }

	@Override
  protected void service(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
	  boolean isMultiPart = FileUploadBase
	      .isMultipartContent(new ServletRequestContext(request));
	 
	  if(isMultiPart) {
	    FileItemFactory factory = new DiskFileItemFactory();
	    ServletFileUpload upload = new ServletFileUpload(factory);
	 
	    try {
	      @SuppressWarnings("rawtypes")
        List items = upload.parseRequest(request);
	      
	      File file = null;
	      boolean debugEnabled = false;
	      Vector obsCov = null;
	      int obsCovDof = 0;
	      Vector onRoadStateCov = null;
	      int onRoadCovDof = 0;
	      Vector offRoadStateCov = null;
	      int offRoadCovDof = 0;
	      Vector offProbs = null;
	      Vector onProbs = null;
	      String filterTypeName = null;
	      int numParticles = 0;
	      int initialObsFreq = 0;
	      long seed = 0l;
	      
	      Iterator iter = items.iterator();
	      while(iter.hasNext()) {
	        
  	      FileItem uploadedFileItem = (FileItem) iter.next();
  	 
  	      if (uploadedFileItem.isFormField()) {
  	        
  	        if (uploadedFileItem.getFieldName().equalsIgnoreCase("debugEnabled")) {
              debugEnabled = Boolean.parseBoolean(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("obsCov")) {
  	          obsCov = parseVector(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("obsCovDof")) {
              obsCovDof = Integer.parseInt(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("onRoadStateCov")) {
  	          onRoadStateCov = parseVector(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("onRoadCovDof")) {
              onRoadCovDof = Integer.parseInt(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("offRoadStateCov")) {
  	          offRoadStateCov = parseVector(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("offRoadCovDof")) {
              offRoadCovDof = Integer.parseInt(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("offProbs")) {
  	          offProbs= parseVector(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("onProbs")) {
  	          onProbs= parseVector(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("filterTypeName")) {
  	          filterTypeName = uploadedFileItem.getString();
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("numParticles")) {
              numParticles = Integer.parseInt(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("initialObsFreq")) {
              initialObsFreq = Integer.parseInt(uploadedFileItem.getString());
  	        } else if (uploadedFileItem.getFieldName().equalsIgnoreCase("seed")) {
              seed = Long.parseLong(uploadedFileItem.getString());
  	        }
  	      } else {
  	        file = new File("/tmp/upload.csv"); 
  	        FileOutputStream output = new FileOutputStream(file);
  	        IOUtils.copy(uploadedFileItem.getInputStream(), output);
  	      }
	      }
	 
        VehicleStateInitialParameters vehicleStateParams = new VehicleStateInitialParameters(
            obsCov, obsCovDof, onRoadStateCov, onRoadCovDof, offRoadStateCov, 
            offRoadCovDof, offProbs, onProbs, filterTypeName, numParticles, 
            initialObsFreq, seed);
	      TraceParameters parameters = new TraceParameters(file, vehicleStateParams, debugEnabled);
	      
	      readCsv(parameters);
	      
	    } catch(Exception e) {
	      log.error(e.getMessage());
	    }
	    
	  } else {
	    super.service(request, response);
	    return;
	  }
	}
	
	private Vector parseVector(String string) {
    final String[] strValues = string.split(",");
    
    double[] values = new double[strValues.length];
    int i = 0;
    for (String val : strValues) {
      values[i] = Double.parseDouble(val);
      i++;
    }
    
    return VectorFactory.getDefault().copyArray(values);
  }

  Logger log = Logger.getLogger(TraceUploadServiceImpl.class);
	
	final private void readCsv(TraceParameters traceParams) throws IOException, InterruptedException {
	  
    final CSVReader gps_reader =
        new CSVReader(new FileReader(traceParams.getDest()), ';');

    final String fileName = traceParams.getDest().getName();
    log.info("processing gps data from " + fileName);

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
          
          /*
           * Clear previous trace runs
           */
          if (!vehicleId.contains(vehicleId)) {
            InferenceServiceImpl.remove(vehicleId);
            Observation.remove(vehicleId);
          }

          final String lat = line[4];
          final String lng = line[5];
          
          final Observation obs =
              Observation.createObservation(vehicleId, line[6],
                  lat, lng, line[7], null, null);
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
            : InferenceServiceImpl.defaultInfoLevel;

    InferenceServiceImpl.processRecords(observations,
        traceParams.getVehicleStateInitialParams(), traceParams.getFilterTypeName(), level);
    InferenceServiceImpl.getExecutor().awaitTermination(5,
        TimeUnit.SECONDS);

    log.info("finished processing " + fileName);

    for (final String vehicleId : vehicleIds) {
      final InferenceInstance ie =
          InferenceServiceImpl.getInferenceInstance(vehicleId);
      if (ie != null && ie.getAverager().getMean() != null)
        log.info("avg. records per sec = " + 1000d
            / ie.getAverager().getMean().value);

    }
  }
}

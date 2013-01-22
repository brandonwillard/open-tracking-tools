package org.opentrackingtools.impl.statistics.filters;

import gov.sandia.cognition.statistics.DataDistribution;

import java.util.Collections;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.WrappedWeightedValue;
import org.opentrackingtools.impl.graph.paths.EdgePredictiveResults;
import org.opentrackingtools.impl.graph.paths.InferredPath;
import org.opentrackingtools.impl.graph.paths.InferredPathPrediction;
import org.opentrackingtools.impl.graph.paths.PathEdge;

import com.google.common.base.Objects;
import com.google.common.collect.HashBasedTable;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Multimap;
import com.google.common.collect.Table;

public class FilterInformation {

  private final Set<InferredPath> evaluatedPaths;
  private final DataDistribution<VehicleState> resampleDist;
  private final Table<VehicleState, PathEdge, EdgePredictiveResults> stateToPathPreds;

  @SuppressWarnings("unchecked")
  public FilterInformation(
    Set<InferredPath> evaluatedPaths,
    DataDistribution<VehicleState> resampleDist, 
    Multimap<VehicleState, WrappedWeightedValue<InferredPathPrediction>> stateToPathValues) {
    this.evaluatedPaths =
        (Set<InferredPath>) Objects.firstNonNull(
            evaluatedPaths, Collections.emptySet());
    this.resampleDist = resampleDist;
    
    if (stateToPathValues != null) {
      this.stateToPathPreds = HashBasedTable.create();
      for (Entry<VehicleState, WrappedWeightedValue<InferredPathPrediction>> entry : 
        stateToPathValues.entries()) {
        for (Entry<PathEdge, EdgePredictiveResults> mapEntry : 
          entry.getValue().getValue().getEdgeToPredictiveBelief().entrySet()) {
          this.stateToPathPreds.put(entry.getKey(), mapEntry.getKey(), mapEntry.getValue());
        }
      }
    } else {
      this.stateToPathPreds = null;
    }
  }

  public Set<InferredPath> getEvaluatedPaths() {
    return evaluatedPaths;
  }

  public DataDistribution<VehicleState> getResampleDist() {
    return resampleDist;
  }

  public
      EdgePredictiveResults
      getEdgePredictiveResult(VehicleState vehicleState, PathEdge pathEdge) {
    if (stateToPathPreds != null) {
      return stateToPathPreds.get(vehicleState, pathEdge);
    } else {
      return null;
    }
  }

}

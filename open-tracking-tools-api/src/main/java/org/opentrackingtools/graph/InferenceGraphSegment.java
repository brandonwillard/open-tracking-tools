package org.opentrackingtools.graph;

import org.apache.commons.lang3.builder.ToStringBuilder;

import com.google.common.collect.ComparisonChain;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LinearLocation;

public class InferenceGraphSegment implements
    Comparable<InferenceGraphSegment> {

  final protected LinearLocation endIndex;
  final protected LineSegment line;
  final protected InferenceGraphSegment nextSegment;
  final protected InferenceGraphEdge parentEdge;
  final protected double startDistance;
  final protected LinearLocation startIndex;

  public InferenceGraphSegment(LineSegment line,
    InferenceGraphEdge infEdge, InferenceGraphSegment nextSegment) {
    this.line = line;
    this.parentEdge = infEdge;
    this.startIndex =
        infEdge.getLocationIndexedLine().indexOf(line.p0);
    this.endIndex =
        infEdge.getLocationIndexedLine().indexOfAfter(line.p1,
            this.startIndex);
    this.startDistance =
        infEdge.getLengthLocationMap().getLength(this.startIndex);
    this.nextSegment = nextSegment;
  }

  @Override
  public int compareTo(InferenceGraphSegment o) {
    return ComparisonChain.start()
        .compare(this.startDistance, o.startDistance)
        .compare(this.parentEdge, o.parentEdge).result();
  }

  public LinearLocation getEndIndex() {
    return this.endIndex;
  }

  public LineSegment getLine() {
    return this.line;
  }

  public InferenceGraphSegment getNextSegment() {
    return this.nextSegment;
  }

  public InferenceGraphEdge getParentEdge() {
    return this.parentEdge;
  }

  public double getStartDistance() {
    return this.startDistance;
  }

  public LinearLocation getStartIndex() {
    return this.startIndex;
  }

  @Override
  public String toString() {
    final ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("line", this.line);
    builder.append("parentEdge", this.parentEdge.getEdgeId());
    builder.append("startIndex", this.startIndex.getSegmentIndex());
    return builder.toString();
  }

}

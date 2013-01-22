package org.openplans.tools.tracking.shared;

import com.google.gwt.core.client.JavaScriptObject;

public class ObservationJso extends JavaScriptObject {

  protected ObservationJso() {}
  
  public final native String getVehicleId() /*-{return this.vehicleId;}-*/;
}

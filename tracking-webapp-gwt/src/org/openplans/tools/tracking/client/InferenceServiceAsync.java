package org.openplans.tools.tracking.client;

import java.util.Set;

import com.google.gwt.user.client.rpc.AsyncCallback;

public interface InferenceServiceAsync {

  void onReceive(String obs, AsyncCallback<Void> callback);

  void getFilterTypes(AsyncCallback<Set<String>> callback);

}

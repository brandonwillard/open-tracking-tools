package org.opentrackingtools.webapp.client;

import java.util.Set;

import com.google.gwt.user.client.rpc.RemoteService;
import com.google.gwt.user.client.rpc.RemoteServiceRelativePath;

@RemoteServiceRelativePath("inferenceService")
public interface InferenceService extends RemoteService {

	void onReceive(String obs) throws Exception;
	Set<String> getFilterTypes();

}

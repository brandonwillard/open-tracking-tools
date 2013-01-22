package org.openplans.tools.tracking.client;

import java.util.Set;

import com.google.gwt.core.client.EntryPoint;
import com.google.gwt.core.client.GWT;
import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.CheckBox;
import com.google.gwt.user.client.ui.FileUpload;
import com.google.gwt.user.client.ui.FormPanel;
import com.google.gwt.user.client.ui.FormPanel.SubmitCompleteEvent;
import com.google.gwt.user.client.ui.FormPanel.SubmitCompleteHandler;
import com.google.gwt.user.client.ui.FormPanel.SubmitHandler;
import com.google.gwt.user.client.ui.HorizontalPanel;
import com.google.gwt.user.client.ui.Label;
import com.google.gwt.user.client.ui.ListBox;
import com.google.gwt.user.client.ui.RootPanel;
import com.google.gwt.user.client.ui.TextBox;
import com.google.gwt.user.client.ui.VerticalPanel;
import com.google.gwt.user.client.ui.FormPanel.SubmitEvent;
import com.gwtext.client.data.ArrayReader;
import com.gwtext.client.data.FieldDef;
import com.gwtext.client.data.MemoryProxy;
import com.gwtext.client.data.Reader;
import com.gwtext.client.data.RecordDef;
import com.gwtext.client.data.Store;
import com.gwtext.client.data.StringFieldDef;
import com.gwtext.client.widgets.grid.BaseColumnConfig;
import com.gwtext.client.widgets.grid.ColumnConfig;
import com.gwtext.client.widgets.grid.ColumnModel;
import com.gwtext.client.widgets.grid.GridPanel;

/**
 * Entry point classes define <code>onModuleLoad()</code>.
 */
public class Tracking_webapp_gwt implements EntryPoint {
	/**
	 * The message displayed to the user when the server cannot be reached or
	 * returns an error.
	 */
	private static final String SERVER_ERROR = "An error occurred while "
			+ "attempting to contact the server. Please check your network "
			+ "connection and try again.";
	
	private final InferenceServiceAsync inferenceService = GWT
			.create(InferenceService.class);
	
	private final TraceUploadServiceAsync traceUploadService = GWT
			.create(TraceUploadService.class);
	
	public static final class InstancesGrid extends GridPanel {
	  
  	private final RecordDef recordDef = new RecordDef(
  	    new FieldDef[] {
  	      new StringFieldDef("id"),
  	      new StringFieldDef("time")
        });
  	
  	private BaseColumnConfig[] columns = new BaseColumnConfig[] {
  	    new ColumnConfig("Id", "id", 160, true),
  	    new ColumnConfig("Time", "time", 160, true)
  	};
  	
    public InstancesGrid() {  
      
      Object[][] data = new Object[][] {};  
      MemoryProxy proxy = new MemoryProxy(data);  

      ArrayReader reader = new ArrayReader(recordDef);  
      Store store = new Store(proxy, reader);  
      store.load();  
      setStore(store);  

      ColumnModel columnModel = new ColumnModel(columns);  
      setColumnModel(columnModel);  

      setFrame(true);  
      setStripeRows(true);  
      setAutoExpandColumn("id");  
    }
	}
	    
	private FormPanel createUploadForm() {
	  final FormPanel form = new FormPanel();
	  form.setAction(GWT.getModuleBaseURL() + "traceUpload");
	  form.setEncoding(FormPanel.ENCODING_MULTIPART);
    form.setMethod(FormPanel.METHOD_POST);
    
    VerticalPanel panel = new VerticalPanel();
    form.setWidget(panel);
    
    final FileUpload upload = new FileUpload();
    upload.setName("uploadFormElement");
    panel.add(upload);
    
    final ListBox lb = new ListBox();
    lb.setName("filterTypeName");
    
    inferenceService.getFilterTypes(new AsyncCallback<Set<String>>() {
      @Override
      public void onFailure(Throwable caught) {
      }
      @Override
      public void onSuccess(Set<String> result) {
        for (String name : result) {
          lb.addItem(name);
        }
      }
    });
    
    panel.add(lb); 
    
		HorizontalPanel obsPanel = new HorizontalPanel();
		final Label obsCovLabel = new Label("obs cov");
		obsPanel.add(obsCovLabel);
		final TextBox obsCovField = new TextBox();
		obsCovField.setName("obsCov");
		obsCovField.setText("100, 100");
		obsPanel.add(obsCovField);
		panel.add(obsPanel);
		
		HorizontalPanel obsDofPanel = new HorizontalPanel();
		final Label obsCovDofLabel = new Label("obs cov dof");
		obsDofPanel.add(obsCovDofLabel);
		final TextBox obsCovDofField = new TextBox();
		obsCovDofField.setName("obsCovDof");
		obsCovDofField.setText("20");
		obsDofPanel.add(obsCovDofField);
		panel.add(obsDofPanel);
		
		HorizontalPanel onRoadStateCovPanel = new HorizontalPanel();
		final Label onRoadStateCovLabel = new Label("on-road cov");
		onRoadStateCovPanel.add(onRoadStateCovLabel);
		final TextBox onRoadStateCovField = new TextBox();
		onRoadStateCovField.setName("onRoadStateCov");
		onRoadStateCovField.setText("6.25e-4");
		onRoadStateCovPanel.add(onRoadStateCovField);
		panel.add(onRoadStateCovPanel);
		
		HorizontalPanel onRoadCovDofPanel = new HorizontalPanel();
		final Label onRoadCovDofLabel = new Label("on-road cov dof");
		onRoadCovDofPanel.add(onRoadCovDofLabel);
		final TextBox onRoadCovDofField = new TextBox();
		onRoadCovDofField.setName("onRoadCovDof");
		onRoadCovDofField.setText("20");
		onRoadCovDofPanel.add(onRoadCovDofField);
		panel.add(onRoadCovDofPanel);
		
		HorizontalPanel offRoadStateCovPanel = new HorizontalPanel();
		final Label offRoadStateCovLabel = new Label("off-road cov");
		offRoadStateCovPanel.add(offRoadStateCovLabel);
		final TextBox offRoadStateCovField = new TextBox();
		offRoadStateCovField.setName("offRoadStateCov");
		offRoadStateCovField.setText("6.25e-4, 6.25e-4");
		offRoadStateCovPanel.add(offRoadStateCovField);
		panel.add(offRoadStateCovPanel);
		
		HorizontalPanel offRoadCovDofPanel = new HorizontalPanel();
		final Label offRoadCovDofLabel = new Label("off-road cov dof");
		offRoadCovDofPanel.add(offRoadCovDofLabel);
		final TextBox offRoadCovDofField = new TextBox();
		offRoadCovDofField.setName("offRoadCovDof");
		offRoadCovDofField.setText("20");
		offRoadCovDofPanel.add(offRoadCovDofField);
		panel.add(offRoadCovDofPanel);
		
		HorizontalPanel offProbsPanel = new HorizontalPanel();
		final Label offProbsLabel = new Label("off-road trans");
		offProbsPanel.add(offProbsLabel);
		final TextBox offProbsField = new TextBox();
		offProbsField.setName("offProbs");
		offProbsField.setText("5, 95");
		offProbsPanel.add(offProbsField);
		panel.add(offProbsPanel);
		
		HorizontalPanel onProbsPanel = new HorizontalPanel();
		final Label onProbsLabel = new Label("on-road trans");
		onProbsPanel.add(onProbsLabel);
		final TextBox onProbsField = new TextBox();
		onProbsField.setName("onProbs");
		onProbsField.setText("95, 5");
		onProbsPanel.add(onProbsField);
		panel.add(onProbsPanel);
		
		HorizontalPanel debugEnabledPanel = new HorizontalPanel();
		final Label debugEnabledLabel = new Label("debug enabled");
		debugEnabledPanel.add(debugEnabledLabel);
		final CheckBox debugEnabledField = new CheckBox();
		debugEnabledField.setName("debugEnabled");
		debugEnabledField.setValue(true);
		debugEnabledPanel.add(debugEnabledField);
		panel.add(debugEnabledPanel);
		
		HorizontalPanel numParticlesPanel = new HorizontalPanel();
		final Label numParticlesLabel = new Label("num particles");
		numParticlesPanel.add(numParticlesLabel);
		final TextBox numParticlesField = new TextBox();
		numParticlesField.setName("numParticles");
		numParticlesField.setText("25");
		numParticlesPanel.add(numParticlesField);
		panel.add(numParticlesPanel);
		
		HorizontalPanel initialObsFreqPanel = new HorizontalPanel();
		final Label initialObsFreqLabel = new Label("initial obs freq");
		initialObsFreqPanel.add(initialObsFreqLabel);
		final TextBox initialObsFreqField = new TextBox();
		initialObsFreqField.setName("initialObsFreq");
		initialObsFreqField.setText("30");
		initialObsFreqPanel.add(initialObsFreqField);
		panel.add(initialObsFreqPanel);
		
		HorizontalPanel seedPanel = new HorizontalPanel();
		final Label seedLabel = new Label("seed");
		seedPanel.add(seedLabel);
		final TextBox seedField = new TextBox();
		seedField.setName("seed");
		seedField.setText("-1");
		seedPanel.add(seedField);
		panel.add(seedPanel);
		
    panel.add(new Button("Submit", new ClickHandler() {
      @Override
      public void onClick(ClickEvent event) {
        form.submit();
      }
    }));
    
    form.addSubmitHandler(new SubmitHandler() {
      @Override
      public void onSubmit(SubmitEvent event) {
        // TODO
      }
    });
    
    form.addSubmitCompleteHandler(new SubmitCompleteHandler() {
      @Override
      public void onSubmitComplete(SubmitCompleteEvent event) {
//        Window.alert(event.getResults()); 
      }
    });
    
    return form;
	}

	/**
	 * This is the entry point method.
	 */
	@Override
  public void onModuleLoad() {
		
	  InstancesGrid instancesGrid = new InstancesGrid();
	  
		RootPanel.get().add(this.createUploadForm());
    RootPanel.get().add(instancesGrid);
	}
}

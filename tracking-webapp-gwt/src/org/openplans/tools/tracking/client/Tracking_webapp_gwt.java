package org.openplans.tools.tracking.client;

import java.io.File;
import java.util.Date;
import java.util.Set;

import com.google.gwt.core.client.EntryPoint;
import com.google.gwt.core.client.GWT;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.RootPanel;

import com.smartgwt.client.data.DSCallback;
import com.smartgwt.client.data.DSResponse;
import com.smartgwt.client.data.DataSource;
import com.smartgwt.client.data.DSRequest;
import com.smartgwt.client.data.fields.DataSourceBinaryField;
import com.smartgwt.client.data.fields.DataSourceBooleanField;
import com.smartgwt.client.data.fields.DataSourceDateField;
import com.smartgwt.client.data.fields.DataSourceEnumField;
import com.smartgwt.client.data.fields.DataSourceFloatField;
import com.smartgwt.client.data.fields.DataSourceIntegerField;
import com.smartgwt.client.data.fields.DataSourceTextField;
import com.smartgwt.client.rpc.RPCCallback;
import com.smartgwt.client.rpc.RPCManager;
import com.smartgwt.client.rpc.RPCRequest;
import com.smartgwt.client.rpc.RPCResponse;
import com.smartgwt.client.util.SC;
import com.smartgwt.client.widgets.IButton;
import com.smartgwt.client.widgets.Label;
import com.smartgwt.client.widgets.form.DynamicForm;
import com.smartgwt.client.widgets.events.ClickEvent;
import com.smartgwt.client.widgets.events.ClickHandler;
import com.smartgwt.client.widgets.form.fields.FileItem;
import com.smartgwt.client.widgets.form.validator.FloatPrecisionValidator;
import com.smartgwt.client.widgets.form.validator.FloatRangeValidator;
import com.smartgwt.client.widgets.grid.ListGrid;
import com.smartgwt.client.widgets.grid.ListGridRecord;
import com.smartgwt.client.widgets.layout.VLayout;

/**
 * Entry point classes define <code>onModuleLoad()</code>.
 */
public class Tracking_webapp_gwt implements EntryPoint {

  private final static InferenceServiceAsync inferenceService = GWT
      .create(InferenceService.class);

  private final TraceUploadServiceAsync traceUploadService = GWT
      .create(TraceUploadService.class);

  //	
  //	public static final class InstancesGrid extends GridPanel {
  //	  
  //  	private final Record recordDef = new Record(
  //  	    new FieldDef[] {
  //  	      new StringFieldDef("id"),
  //  	      new StringFieldDef("time")
  //        });
  //  	
  //  	private BaseColumnConfig[] columns = new BaseColumnConfig[] {
  //  	    new ColumnConfig("Id", "id", 160, true),
  //  	    new ColumnConfig("Time", "time", 160, true)
  //  	};
  //  	
  //    public InstancesGrid() {  
  //      
  //      Object[][] data = new Object[][] {};  
  //      MemoryProxy proxy = new MemoryProxy(data);  
  //
  //      ArrayReader reader = new ArrayReader(recordDef);  
  //      Store store = new Store(proxy, reader);  
  //      store.load();  
  //      setStore(store);  
  //
  //      ColumnModel columnModel = new ColumnModel(columns);  
  //      setColumnModel(columnModel);  
  //
  //      setFrame(true);  
  //      setStripeRows(true);  
  //      setAutoExpandColumn("id");  
  //    }
  //	}
  //	    
  //	private FormPanel createUploadForm() {
  //	  final FormPanel form = new FormPanel();
  //	  form.setAction(GWT.getModuleBaseURL() + "traceUpload");
  //	  form.setEncoding(FormPanel.ENCODING_MULTIPART);
  //    form.setMethod(FormPanel.METHOD_POST);
  //    
  //    VerticalPanel panel = new VerticalPanel();
  //    form.setWidget(panel);
  //    
  //    final FileUpload upload = new FileUpload();
  //    upload.setName("uploadFormElement");
  //    panel.add(upload);
  //    
  //    final ListBox lb = new ListBox();
  //    lb.setName("filterTypeName");
  //    
  //    inferenceService.getFilterTypes(new AsyncCallback<Set<String>>() {
  //      @Override
  //      public void onFailure(Throwable caught) {
  //      }
  //      @Override
  //      public void onSuccess(Set<String> result) {
  //        for (String name : result) {
  //          lb.addItem(name);
  //        }
  //      }
  //    });
  //    
  //    panel.add(lb); 
  //    
  //		HorizontalPanel obsPanel = new HorizontalPanel();
  //		final Label obsCovLabel = new Label("obs cov");
  //		obsPanel.add(obsCovLabel);
  //		final TextBox obsCovField = new TextBox();
  //		obsCovField.setName("obsCov");
  //		obsCovField.setText("100, 100");
  //		obsPanel.add(obsCovField);
  //		panel.add(obsPanel);
  //		
  //		HorizontalPanel obsDofPanel = new HorizontalPanel();
  //		final Label obsCovDofLabel = new Label("obs cov dof");
  //		obsDofPanel.add(obsCovDofLabel);
  //		final TextBox obsCovDofField = new TextBox();
  //		obsCovDofField.setName("obsCovDof");
  //		obsCovDofField.setText("20");
  //		obsDofPanel.add(obsCovDofField);
  //		panel.add(obsDofPanel);
  //		
  //		HorizontalPanel onRoadStateCovPanel = new HorizontalPanel();
  //		final Label onRoadStateCovLabel = new Label("on-road cov");
  //		onRoadStateCovPanel.add(onRoadStateCovLabel);
  //		final TextBox onRoadStateCovField = new TextBox();
  //		onRoadStateCovField.setName("onRoadStateCov");
  //		onRoadStateCovField.setText("6.25e-4");
  //		onRoadStateCovPanel.add(onRoadStateCovField);
  //		panel.add(onRoadStateCovPanel);
  //		
  //		HorizontalPanel onRoadCovDofPanel = new HorizontalPanel();
  //		final Label onRoadCovDofLabel = new Label("on-road cov dof");
  //		onRoadCovDofPanel.add(onRoadCovDofLabel);
  //		final TextBox onRoadCovDofField = new TextBox();
  //		onRoadCovDofField.setName("onRoadCovDof");
  //		onRoadCovDofField.setText("20");
  //		onRoadCovDofPanel.add(onRoadCovDofField);
  //		panel.add(onRoadCovDofPanel);
  //		
  //		HorizontalPanel offRoadStateCovPanel = new HorizontalPanel();
  //		final Label offRoadStateCovLabel = new Label("off-road cov");
  //		offRoadStateCovPanel.add(offRoadStateCovLabel);
  //		final TextBox offRoadStateCovField = new TextBox();
  //		offRoadStateCovField.setName("offRoadStateCov");
  //		offRoadStateCovField.setText("6.25e-4, 6.25e-4");
  //		offRoadStateCovPanel.add(offRoadStateCovField);
  //		panel.add(offRoadStateCovPanel);
  //		
  //		HorizontalPanel offRoadCovDofPanel = new HorizontalPanel();
  //		final Label offRoadCovDofLabel = new Label("off-road cov dof");
  //		offRoadCovDofPanel.add(offRoadCovDofLabel);
  //		final TextBox offRoadCovDofField = new TextBox();
  //		offRoadCovDofField.setName("offRoadCovDof");
  //		offRoadCovDofField.setText("20");
  //		offRoadCovDofPanel.add(offRoadCovDofField);
  //		panel.add(offRoadCovDofPanel);
  //		
  //		HorizontalPanel offProbsPanel = new HorizontalPanel();
  //		final Label offProbsLabel = new Label("off-road trans");
  //		offProbsPanel.add(offProbsLabel);
  //		final TextBox offProbsField = new TextBox();
  //		offProbsField.setName("offProbs");
  //		offProbsField.setText("5, 95");
  //		offProbsPanel.add(offProbsField);
  //		panel.add(offProbsPanel);
  //		
  //		HorizontalPanel onProbsPanel = new HorizontalPanel();
  //		final Label onProbsLabel = new Label("on-road trans");
  //		onProbsPanel.add(onProbsLabel);
  //		final TextBox onProbsField = new TextBox();
  //		onProbsField.setName("onProbs");
  //		onProbsField.setText("95, 5");
  //		onProbsPanel.add(onProbsField);
  //		panel.add(onProbsPanel);
  //		
  //		HorizontalPanel debugEnabledPanel = new HorizontalPanel();
  //		final Label debugEnabledLabel = new Label("debug enabled");
  //		debugEnabledPanel.add(debugEnabledLabel);
  //		final CheckBox debugEnabledField = new CheckBox();
  //		debugEnabledField.setName("debugEnabled");
  //		debugEnabledField.setValue(true);
  //		debugEnabledPanel.add(debugEnabledField);
  //		panel.add(debugEnabledPanel);
  //		
  //		HorizontalPanel numParticlesPanel = new HorizontalPanel();
  //		final Label numParticlesLabel = new Label("num particles");
  //		numParticlesPanel.add(numParticlesLabel);
  //		final TextBox numParticlesField = new TextBox();
  //		numParticlesField.setName("numParticles");
  //		numParticlesField.setText("25");
  //		numParticlesPanel.add(numParticlesField);
  //		panel.add(numParticlesPanel);
  //		
  //		HorizontalPanel initialObsFreqPanel = new HorizontalPanel();
  //		final Label initialObsFreqLabel = new Label("initial obs freq");
  //		initialObsFreqPanel.add(initialObsFreqLabel);
  //		final TextBox initialObsFreqField = new TextBox();
  //		initialObsFreqField.setName("initialObsFreq");
  //		initialObsFreqField.setText("30");
  //		initialObsFreqPanel.add(initialObsFreqField);
  //		panel.add(initialObsFreqPanel);
  //		
  //		HorizontalPanel seedPanel = new HorizontalPanel();
  //		final Label seedLabel = new Label("seed");
  //		seedPanel.add(seedLabel);
  //		final TextBox seedField = new TextBox();
  //		seedField.setName("seed");
  //		seedField.setText("-1");
  //		seedPanel.add(seedField);
  //		panel.add(seedPanel);
  //		
  //    panel.add(new Button("Submit", new ClickHandler() {
  //      @Override
  //      public void onClick(ClickEvent event) {
  //        form.submit();
  //      }
  //    }));
  //    
  //    form.addSubmitHandler(new SubmitHandler() {
  //      @Override
  //      public void onSubmit(SubmitEvent event) {
  //        // TODO
  //      }
  //    });
  //    
  //    form.addSubmitCompleteHandler(new SubmitCompleteHandler() {
  //      @Override
  //      public void onSubmitComplete(SubmitCompleteEvent event) {
  ////        Window.alert(event.getResults()); 
  //      }
  //    });
  //    
  //    return form;
  //	}

  /**
   * This is the entry point method.
   */
  @Override
  public void onModuleLoad() {
    VLayout layout = new VLayout(15);
    
    Label label = new Label();
    label.setHeight(10);
    label.setWidth100();
    label.setContents("Showing items in Category 'Rollfix Glue");
    layout.addMember(label);

    final DataSource dataSource = ItemSupplyLocalDS.getInstance();

    ListGrid listGrid = new ListGrid();
    listGrid.setWidth(500);
//    listGrid.setWidth100();
    listGrid.setHeight(200);
    listGrid.setDataSource(dataSource);
    listGrid.setAutoFetchData(true);
    layout.addMember(listGrid);

    final DynamicForm form = new DynamicForm();
    form.setNumCols(4);
    form.setDataSource(dataSource);

    form.setValue("category", "Rollfix Glue");
    form.setValue("itemName", "[Enter Item Name]");
    form.setValue("SKU", "[SKU]");
    form.setValue("unitCoset", "[Enter Price]");

    layout.addMember(form);
    IButton button = new IButton("Save New");
    button.addClickHandler(new ClickHandler() {
      public void onClick(ClickEvent event) {
        form.saveData(new DSCallback() {
          public void execute(DSResponse response, Object rawData,
            DSRequest request) {
            
            form.editNewRecord();
            
            RPCRequest rpcRequest = new RPCRequest();
            
            rpcRequest.setData(request.getData());
            
            rpcRequest.setActionURL(GWT.getModuleBaseURL() + "traceUpload");
           
            RPCManager.sendRequest(rpcRequest, 
                new RPCCallback () {
                    public void execute(RPCResponse response, 
                      Object rawData, RPCRequest request) {
                        SC.say("Response from the server:" + rawData);
                    }
                }
            ); 
          }
        });
        form.reset();
      }
    });
    layout.addMember(button);
    //    RootPanel.get().add(layout);
    layout.draw();
  }

  public static class ItemSupplyLocalDS extends DataSource {

    private static ItemSupplyLocalDS instance = null;

    public static ItemSupplyLocalDS getInstance() {
      if (instance == null) {
        instance = new ItemSupplyLocalDS("supplyItemLocalDS");
      }
      return instance;
    }

    public ItemSupplyLocalDS(String id) {

      setID(id);
      DataSourceIntegerField pkField =
          new DataSourceIntegerField("itemID");
      pkField.setHidden(true);
      pkField.setPrimaryKey(true);

      DataSourceTextField itemNameField =
          new DataSourceTextField("itemName", "Item Name", 128, true);
      DataSourceTextField skuField =
          new DataSourceTextField("SKU", "SKU", 10, true);

      DataSourceTextField descriptionField =
          new DataSourceTextField("description", "Description", 2000);
      DataSourceTextField categoryField =
          new DataSourceTextField("category", "Category", 128, true);
      categoryField.setForeignKey("supplyCategory.categoryName");

      final DataSourceEnumField filtersField =
          new DataSourceEnumField("filterTypeName", "Filters", 5);
      
      inferenceService.getFilterTypes(new AsyncCallback<Set<String>>() {
        @Override
        public void onFailure(Throwable caught) {
        }
        @Override
        public void onSuccess(Set<String> result) {
          filtersField.setValueMap(result.toArray(new String[result.size()]));
        }
      });

      DataSourceFloatField unitCostField =
          new DataSourceFloatField("unitCost", "Unit Cost", 5);
      FloatRangeValidator rangeValidator = new FloatRangeValidator();
      rangeValidator.setMin(0);
      rangeValidator
          .setErrorMessage("Please enter a valid (positive) cost");

      FloatPrecisionValidator precisionValidator =
          new FloatPrecisionValidator();
      precisionValidator.setPrecision(2);
      precisionValidator
          .setErrorMessage("The maximum allowed precision is 2");

      unitCostField.setValidators(rangeValidator, precisionValidator);

      DataSourceBooleanField inStockField =
          new DataSourceBooleanField("inStock", "In Stock");

      DataSourceDateField nextShipmentField =
          new DataSourceDateField("nextShipment", "Next Shipment");
      
      DataSourceBinaryField fileField = 
          new DataSourceBinaryField("uploadFormElement", "Trace", 100);

      setFields(pkField, itemNameField, skuField, descriptionField,
          categoryField, filtersField, unitCostField, inStockField,
          nextShipmentField, fileField);

      setClientOnly(true);
    }
  }

  public static class ItemRecord extends ListGridRecord {

    public ItemRecord() {
    }

    public ItemRecord(int itemID, String item, String sku,
      String description, String category, String filters,
      Double unitCost, Boolean inStock, Date nextShipment,
      FileItem file) {
      setItemID(itemID);
      setItemName(item);
      setSKU(sku);
      setDescription(description);
      setCategory(category);
      setFilter(filters);
      setUnitCost(unitCost);
      setInStock(inStock);
      setNextShipment(nextShipment);
      setFile(file);
    }

    public void setFile(FileItem file) {
      setAttribute("uploadFormElement", file);
    }
    
    public FileItem getFile() {
      return (FileItem) getAttributeAsObject("uploadFormElement");
    }
    
    public void setItemID(int itemID) {
      setAttribute("itemID", itemID);
    }

    public int getItemID() {
      return getAttributeAsInt("itemID");
    }

    public void setItemName(String item) {
      setAttribute("itemName", item);
    }

    public String getItemName() {
      return getAttribute("itemName");
    }

    public void setSKU(String SKU) {
      setAttribute("SKU", SKU);
    }

    public String getSKU() {
      return getAttribute("SKU");
    }

    public void setDescription(String description) {
      setAttribute("description", description);
    }

    public String getDescription() {
      return getAttribute("description");
    }

    public void setCategory(String category) {
      setAttribute("category", category);
    }

    public String getCategory() {
      return getAttribute("category");
    }

    public void setFilter(String filter) {
      setAttribute("filterTypeName", filter);
    }

    public String getFilter() {
      return getAttribute("filterTypeName");
    }

    public void setUnitCost(Double unitCost) {
      setAttribute("unitCost", unitCost);
    }

    public Float getUnitCost() {
      return getAttributeAsFloat("unitCost");
    }

    public void setInStock(Boolean inStock) {
      setAttribute("inStock", inStock);
    }

    public Boolean getInStock() {
      return getAttributeAsBoolean("inStock");
    }

    public void setNextShipment(Date nextShipment) {
      setAttribute("nextShipment", nextShipment);
    }

    public Date getNextShipment() {
      return getAttributeAsDate("nextShipment");
    }

  }
}

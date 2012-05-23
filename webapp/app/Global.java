

import play.*;


public class Global extends GlobalSettings {


  @Override
  public void onStart(Application app) {
	
  }  
  
  @Override
  public void onStop(Application app) {
    Logger.info("Application shutdown...");
  }  
  
  public void doSomething()
  {
	  Logger.info("doing something...");
  }
    
}
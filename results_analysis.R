library("rjson")

getEdgeTransCIResultsJson <- function(simName) {
  json_file <-
    paste("http://localhost:9000/api/getEdgeTransitionStats?vehicleId=", simName,
          sep="")

  json_data <- readLines(json_file)
  #json_data <- sub('[^\\{]*', '', json_data) # remove function name and opening parenthesis
  #json_data <- sub('\\)$', '', json_data) # remove closing parenthesis

  json_data <- fromJSON(paste(json_data, collapse=""))
  return(json_data)
}

getSimBestStatesJson <- function(simName) {
  json_file <-
    paste("http://localhost:9000/api/traces?vehicleId=", simName,
          sep="")

  json_data <- readLines(json_file)
  #json_data <- sub('[^\\{]*', '', json_data) # remove function name and opening parenthesis
  #json_data <- sub('\\)$', '', json_data) # remove closing parenthesis

  json_data <- fromJSON(paste(json_data, collapse=""))
  return(json_data)
}
                                         
getSimPerfJson <- function(simName) {
  json_file <-
    paste("http://localhost:9000/api/getPerformanceResults?vehicleId=", simName,
          sep="")

  json_data <- readLines(json_file)
  json_data <- sub('[^\\{]*', '', json_data) # remove function name and opening parenthesis
  json_data <- sub('\\)$', '', json_data) # remove closing parenthesis

  json_data <- fromJSON(paste(json_data, collapse=""))
  return(json_data$results)
}

rmse.data = data.frame()

#
# PL runs
#
{
  pl_means_25_data = list()
  pl_means_25_data$desc =   
    "# vehicleId: sim-1146299170, recordsProcessed: 374 /1200 
    # isSimulation: true 
    # Initial Parameters:
    # obsCov: <100.0, 100.0>
    # obsCovDof: 20
    # onRoadStateCov: <6.25E-4>
    # onRoadStateCovDof: 20
    # offRoadStateCov: <6.25E-4, 6.25E-4>
    # offRoadStateCovDof: 20
    # offTransitionProbs: <0.05, 1.0>
    # onTransitionProbs: <1.0, 0.05>
    # numParticles: 25
    # initialObsFreq: 30
    # filterTypeName: org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingPLFilter
    # seed: 1234567890"
  pl_means_25_data$id = "sim-1146299170"
  pl_means_25_data$rmse <- unlist(lapply(getSimPerfJson(pl_means_25_data$id), function(x) sqrt(x$mean)))
  pl_means_25_data$best <- getSimBestStatesJson(pl_means_25_data$id)
  save(pl_means_25_data, file=paste(pl_means_25_data$id, "Rdata", sep='.'))

  rbind(rmse.data, 
        data.frame(Time=seq_along(pl_means_25_data$rmse), 
                   Filter=rep("PL", length(pl_means_25_data$rmse)), 
                   Particles=rep(25, length(pl_means_25_data$rmse)), 
                   RMSE=pl_means_25_data$rmse)
        )
}

{
  pl_means_25_2_data = list()
  pl_means_25_2_data$desc =  
   "vehicleId: sim-1394830191, recordsProcessed: 0 /1200 
  isSimulation: true 
  Initial Parameters:
  obsCov: <100.0, 100.0>
  obsCovDof: 20
  onRoadStateCov: <6.25E-4>
  onRoadStateCovDof: 20
  offRoadStateCov: <6.25E-4, 6.25E-4>
  offRoadStateCovDof: 20
  offTransitionProbs: <20.0, 70.0>
  onTransitionProbs: <100.0, 15.0>
  numParticles: 25
  initialObsFreq: 30
  filterTypeName: org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingPLFilter
  seed: 1234567890"
  pl_means_25_2_data$id = "sim-1394830191"
  pl_means_25_2_data$rmse <- 
    unlist(lapply(getSimPerfJson(pl_means_25_2_data$id), function(x) sqrt(x$mean)))
  pl_means_25_2_data$best <- getSimBestStatesJson(pl_means_25_2_data$id)
  save(pl_means_25_2_data, file=paste(pl_means_25_2_data$id, "Rdata", sep='.'))

  rmse.data = rbind(rmse.data, 
        data.frame(Time=seq_along(pl_means_25_2_data$rmse), 
                   Filter=rep("PL-2", length(pl_means_25_2_data$rmse)), 
                   Particles=rep(25, length(pl_means_25_2_data$rmse)), 
                   RMSE=pl_means_25_2_data$rmse)
        )

  plot(pl_means_25_2_data$rmse, type='l')

}

{
  pl_means_50_data = list()
  pl_means_50_data$desc =  
  "vehicleId: sim-2012700520, recordsProcessed: 1200 /1200 
    isSimulation: true 
    Initial Parameters:
    obsCov: <100.0, 100.0>
    obsCovDof: 20
    onRoadStateCov: <6.25E-4>
    onRoadStateCovDof: 20
    offRoadStateCov: <6.25E-4, 6.25E-4>
    offRoadStateCovDof: 20
    offTransitionProbs: <20.0, 70.0>
    onTransitionProbs: <100.0, 15.0>
    numParticles: 50
    initialObsFreq: 30
    filterTypeName: org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingPLFilter
    seed: 1234567890"
  pl_means_50_data$id = "sim-2012700520"
  pl_means_50_data$rmse <- 
    unlist(lapply(getSimPerfJson(pl_means_50_data$id), function(x) sqrt(x$mean)))
  pl_means_50_data$best <- getSimBestStatesJson(pl_means_50_data$id)
  save(pl_means_50_data, file=paste(pl_means_50_data$id, "Rdata", sep='.'))

  rmse.data = rbind(rmse.data, 
        data.frame(Time=seq_along(pl_means_50_data$rmse), 
                   Filter=rep("PL-2", length(pl_means_50_data$rmse)), 
                   Particles=rep(50, length(pl_means_50_data$rmse)), 
                   RMSE=pl_means_50_data$rmse)
        )

  plot(pl_means_25_2_data$rmse, type='l')
  lines(pl_means_50_data$rmse, col="red")

}


#
# BS runs
#
{
 bs_means_50_data = list()
 bs_means_50_data$desc = 
   "vehicleId: sim2082684340, recordsProcessed: 1200 /1200 
   isSimulation: true 
   Initial Parameters:
   obsCov: <100.0, 100.0>
   obsCovDof: 20
   onRoadStateCov: <6.25E-4>
   onRoadStateCovDof: 20
   offRoadStateCov: <6.25E-4, 6.25E-4>
   offRoadStateCovDof: 20
   offTransitionProbs: <0.05, 1.0>
   onTransitionProbs: <1.0, 0.05>
   numParticles: 50
   initialObsFreq: 30
   filterTypeName: org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingBootstrapFilter
   seed: 1234567890"
  bs_means_50_data$id = "sim2082684340";
  bs_means_50_data$rmse <- unlist(lapply(getSimPerfJson(bs_means_50_data$id), function(x) sqrt(x$mean))) 
  bs_means_50_data$best <- getSimBestStatesJson(bs_means_50_data$id)
  save(bs_means_50_data, file=paste(bs_means_50_data$id, "Rdata", sep='.'))

  rmse.data = rbind(rmse.data, 
        data.frame(Time=seq_along(bs_means_50_data$rmse), 
                   Filter=rep("BS", length(bs_means_50_data$rmse)), 
                   Particles=rep(50, length(bs_means_50_data$rmse)), 
                   RMSE=bs_means_50_data$rmse)
        )

  plot(log1p(pl_means_25_2_data$rmse), type='l')
  lines(log1p(pl_means_50_data$rmse), col="blue")
  lines(log1p(bs_means_50_data$rmse), col="red")
}

{
 bs_means_200_data = list()
 bs_means_200_data$desc = 
   "vehicleId: sim-1624537634, recordsProcessed: 1 /1200 
   isSimulation: true 
   Initial Parameters:
   obsCov: <100.0, 100.0>
   obsCovDof: 20
   onRoadStateCov: <6.25E-4>
   onRoadStateCovDof: 20
   offRoadStateCov: <6.25E-4, 6.25E-4>
   offRoadStateCovDof: 20
   offTransitionProbs: <0.05, 1.0>
   onTransitionProbs: <1.0, 0.05>
   numParticles: 200
   initialObsFreq: 30
   filterTypeName: org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingBootstrapFilter
   seed: 1234567890"
  bs_means_200_data$id = "sim-1624537634";
  bs_means_200_data$rmse <- unlist(lapply(getSimPerfJson(bs_means_200_data$id), function(x) sqrt(x$mean))) 
  bs_means_200_data$best <- getSimBestStatesJson(bs_means_200_data$id)
  save(bs_means_200_data, file=paste(bs_means_200_data$id, "Rdata", sep='.'))

  rmse.data = rbind(rmse.data, 
        data.frame(Time=seq_along(bs_means_200_data$rmse), 
                   Filter=rep("BS", length(bs_means_200_data$rmse)), 
                   Particles=rep(200, length(bs_means_200_data$rmse)), 
                   RMSE=bs_means_200_data$rmse)
        )

  plot(log1p(pl_means_25_2_data$rmse), type='l')
  lines(log1p(pl_means_50_data$rmse), col="blue")
  lines(log1p(bs_means_50_data$rmse), col="orange")
  lines(log1p(bs_means_200_data$rmse), col="red")
}


plot(pl_means_50_est_s1, type='l', log='')
matplot(cbind(pl_means_50_est_s1, pl_means_50_s1, bs_means_50_s1, bs_means_200_s1), type='l', log='')
boxplot(log1p(cbind(pl_means_50_est_s1, pl_means_50_s1, bs_means_50_s1, bs_means_200_s1)), log='')
abline(b=0, a=0)

library(ggplot2)
data = rmse.data
data$partFilter = interaction(data$Filter, data$Particles)

pdf(file="plbs-rmse-1200-wisker.pdf")
  p2 <- ggplot(data, aes(partFilter, RMSE)) + geom_boxplot() + scale_y_log10() +
        theme_bw(base_size=10) + scale_x_discrete(name="")                            
  print(p2)
dev.off()


pdf(file="plbs-rmse-1200-series.pdf", pointsize=7) 
  p1 <- ggplot(data, aes(x=Time, y=RMSE, colour=Particles, group=Filter)) +
        geom_line(alpha=0.7) + facet_grid( Particles ~ Filter) + scale_y_continuous() +
        theme_bw(base_size=10)
  plot(p1)
dev.off()


sim_res <- pl_means_25_2_data$best

# simple on/off-road measure 
sim_res_on_correct <-  unlist(lapply(sim_res, function(x) {
                     return((x$actualResults$inferredEdge$id != -1 
                             && x$infResults$inferredEdge$id != -1) 
                     || (x$actualResults$inferredEdge$id == -1 
                         && x$infResults$inferredEdge$id == -1))
                    }))
mean(sim_res_on_correct)

library(plyr)

#
# creates data frames for a matrix to be plotted
#
matrix_record_maker <- function(time, type, matrix) {
  n = length(matrix)/2;
  if (n == 1/2)
    indx = 1
  else
    indx = expand.grid(col=1:n, row=1:n);
  timestr =  as.POSIXct(as.numeric(time)/1000, origin="1970-01-01", tz="GMT");
  df = data.frame(time=timestr, 
                         type,
                         indx,
                         matrix,
                         check.rows=F, check.names=F);
  return(df);
}

sim_res_obs_cov <- ldply(sim_res, function(x) {
                         return(rbind(matrix_record_maker(x$time, "inferred", x$infResults$obsCovariance),
                                      matrix_record_maker(x$time, "actual", x$actualResults$obsCovariance)))
                    })

sim_res_obs_prior_mean_cov <- ldply(sim_res, function(x) {
                         return(rbind(matrix_record_maker(x$time, "inferred",
                                x$infResults$obsCovarPrior$scale
                                  /(x$infResults$obsCovarPrior$dof -2 -1)),
                                      matrix_record_maker(x$time, "actual", x$actualResults$obsCovariance)))
                    })

sim_res_onroad_prior_mean_cov <- ldply(sim_res, function(x) {
   return(rbind(matrix_record_maker(x$time, "inferred",
          x$infResults$onRoadCovarPrior$scale
            /(x$infResults$onRoadCovarPrior$dof-1-1)),
                matrix_record_maker(x$time, "actual", 
                  x$actualResults$stateQrCovariance)))
})

sim_res_offroad_prior_mean_cov <- ldply(sim_res, function(x) {
   return(rbind(matrix_record_maker(x$time, "inferred",
          x$infResults$offRoadCovarPrior$scale
            /(x$infResults$offRoadCovarPrior$dof -2 -1)),
                matrix_record_maker(x$time, "actual",
                  x$actualResults$stateQgCovariance)))
})

sim_res_offroad_cov <- ldply(sim_res, function(x) {
  if (x$actualResults$inferredEdge$id == -1 
       && x$infResults$inferredEdge$id == -1) {                         
    return(rbind(matrix_record_maker(x$time, "inferred",
                                     x$infResults$stateCovariance),
                  matrix_record_maker(x$time, "actual",
                                      x$actualResults$stateCovariance)))
  } else {
    return(NULL)
  }
})

sim_res_onroad_cov <- ldply(sim_res, function(x) {

                        if (x$actualResults$inferredEdge$id != -1 
                             && x$infResults$inferredEdge$id != -1) {                         
                          return(rbind(matrix_record_maker(x$time, "inferred",
                                                           x$infResults$stateCovariance),
                                        matrix_record_maker(x$time, "actual",
                                                            x$actualResults$stateCovariance)))
                        } else {
                          return(NULL)
                        }
                    })

sim_res_qr_cov <- ldply(sim_res, function(x) {
 return(rbind(matrix_record_maker(x$time, "inferred",
                x$infResults$stateQrCovariance),
              matrix_record_maker(x$time, "actual", 
                x$actualResults$stateQrCovariance)))
})

sim_res_qg_cov <- ldply(sim_res, function(x) {
                         return(rbind(matrix_record_maker(x$time, "inferred",
                                                          x$infResults$stateQgCovariance),
                                      matrix_record_maker(x$time, "actual", 
                                                          x$actualResults$stateQgCovariance)))
                    })

#i = 1050
#infTransMat = matrix(c(sim_res[[i]]$infResults$offRoadTransProbs,
#  rev(sim_res[[i]]$infResults$onRoadTransProbs)), 2,2, byrow=T)
#colnames(infTransMat) <- c("off", "on")
#rownames(infTransMat) <- c("off", "on")
#
#actTransMat = matrix(c(sim_res[[i]]$actualResults$offRoadTransProbs,
#  rev(sim_res[[i]]$actualResults$onRoadTransProbs)), 2,2, byrow=T)
#colnames(actTransMat) <- c("off", "on")
#rownames(actTransMat) <- c("off", "on")
#
#infTransMat
#actTransMat

sim_onOff_trans_probs <- ldply(getEdgeTransCIResultsJson("sim-2012700520"), function(x) {
   timestr =  as.POSIXct(as.numeric(x$time)/1000, origin="1970-01-01", tz="GMT");
   a1 = data.frame(time=timestr, type="actual",
                    trans="off", value.type="mean", value=x$actualFreeDiagonal,
                    upper=NA, lower=NA);
   a2 = data.frame(time=timestr, type="actual",
                    trans="on", value.type="mean", value=x$actualEdgeDiagonal,
                    upper=NA, lower=NA); 

   f1 = data.frame(time=timestr, type="inf",
                    trans="off", value.type="mean", value=x$infFreeDiagonalMean,
                    upper=x$infFreeDiagonalUpper, lower=x$infFreeDiagonalLower);
   #f2 = data.frame(time=timestr, type="inf",
   #                 trans="off", value.type="upper", value=x$infFreeDiagonalUpper); 
   #f3 = data.frame(time=timestr, type="inf",
   #                 trans="off", value.type="lower", value=x$infFreeDiagonalLower); 

   e1 = data.frame(time=timestr, type="inf",
                    trans="on", value.type="mean", value=x$infEdgeDiagonalMean,
                    upper=x$infEdgeDiagonalUpper, lower=x$infEdgeDiagonalLower);
   #e2 = data.frame(time=timestr, type="inf",
   #                 trans="on", value.type="upper", value=x$infEdgeDiagonalUpper); 
   #e3 = data.frame(time=timestr, type="inf",
   #                 trans="on", value.type="lower", value=x$infEdgeDiagonalLower); 

   #return(rbind(a1, a2, f1, f2, f3, e1, e2, e3));
   return(rbind(a1, a2, f1, e1));
         
})

# Debug
{
  head(sim_res_obs_cov, 8*2)
  tail(sim_res_obs_cov, 8*2)
  range(sim_res_obs_cov$time)

  sim_obs_scales <- ldply(sim_res, function(x) {
    timestr =  as.POSIXct(as.numeric(x$time)/1000, origin="1970-01-01", tz="GMT");
    return( data.frame(time=timestr,
      scale=max(eigen(matrix(x$infResults$obsCovarPrior$scale, 2, 2), 
        symmetric=T, only.values = T)$values)
    ))
  })

  scale_diffs = data.frame(
    time=sim_obs_scales$time[-1],
    diffs=diff(sim_obs_scales$scale))
  head(scale_diffs)
  scale_diffs_plot <- ggplot(scale_diffs, aes(x=time, y=scale)) +
        geom_line(alpha=1.0) + scale_y_continuous() + 
        theme_bw(base_size=10)
  plot(scale_diffs_plot)

  biggest.diff = which.max(abs(diff(sim_obs_scales)))
  sim_res[[biggest.diff+1]]$time

  state_diffs <- ldply(sim_res, function(x) {
    timestr =  as.POSIXct(as.numeric(x$time)/1000, origin="1970-01-01", tz="GMT");
    locdiff = c(x$observedPoint$x, x$observedPoint$y) -
      x$infResults$stateSample$stateLoc;
    return( data.frame(time=timestr,
      diff= sqrt(locdiff %*% locdiff)
    ))
  })
  mean(state_diffs$diff)
  plot(state_diffs, type='l')
}

library(ggplot2)

pdf(file="pl-edge-trans-1200-95CI.pdf", pointsize=7) 

sim_onOff_trans_plot <- ggplot(sim_onOff_trans_probs, aes(x=time, y=value, colour=type,
                           group=interaction(value.type, type))) +
      geom_line(alpha=1.0) + facet_grid(trans ~ ., scales="free") + scale_y_continuous() + 
      theme_bw(base_size=10) + geom_ribbon(aes(ymin=lower, ymax=upper, fill=type), 
                                           alpha=0.2, colour=NA)
plot(sim_onOff_trans_plot)

dev.off()

sim_obs_prior_mean_plot <- ggplot(sim_res_obs_prior_mean_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") + scale_y_continuous() + 
      theme_bw(base_size=10)
plot(sim_obs_prior_mean_plot)

sim_onroad_prior_mean_plot <- ggplot(sim_res_onroad_prior_mean_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) +
      scale_y_log10() + 
      theme_bw(base_size=10)
plot(sim_onroad_prior_mean_plot)

sim_offroad_prior_mean_plot <- ggplot(sim_res_offroad_prior_mean_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") +
      scale_y_log10() + 
      theme_bw(base_size=10)
plot(sim_offroad_prior_mean_plot)

sim_obs_cov_plot <- ggplot(sim_res_obs_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") + scale_y_continuous() + 
      theme_bw(base_size=10)
plot(sim_obs_cov_plot)     

sim_obs_qr_plot <- ggplot(sim_res_qr_cov, 
  aes(x=time, y=matrix, colour=type, group=type)) +
  geom_line(alpha=1.0) + scale_y_continuous() + 
  theme_bw(base_size=10)
plot(sim_obs_qr_plot)     

sim_obs_qg_plot <- ggplot(sim_res_qg_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + 
      facet_grid(row ~ col, scales="free") + scale_y_continuous() + 
      theme_bw(base_size=10)
plot(sim_obs_qg_plot)     

sim_onroad_cov_plot <- ggplot(sim_res_onroad_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") + scale_y_continuous() +
      theme_bw(base_size=10)
plot(sim_onroad_cov_plot)     

sim_offroad_cov_plot <- ggplot(sim_res_offroad_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") + scale_y_log10() +
      theme_bw(base_size=10)
plot(sim_offroad_cov_plot)     


library("rjson")

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

# avg. records per sec = 67.35140596059942
pl_means_25_s1 <- unlist(lapply(getSimPerfJson("sim-1200318255"), function(x) sqrt(x$mean)))

plot(pl_means_25_s1, type='l') 
# avg. records per sec = 39.146604032100214  
pl_means_50_s1 <- unlist(lapply(getSimPerfJson("sim14917717"), function(x) sqrt(x$mean)))

# avg. records per sec = 11.59386684443929  
pl_means_200_s1 <- unlist(lapply(getSimPerfJson("sim112077992"), function(x) sqrt(x$mean)))

# avg. records per sec = 5.587866878384734 
#pl_means_500_s1 <- unlist(lapply(getSimPerfJson("sim75209940"), function(x) sqrt(x$mean)))

# avg. records per sec = 1673.6401673640166
bs_means_50_s1 <- unlist(lapply(getSimPerfJson("sim1391934297"), function(x) sqrt(x$mean))) 
plot(bs_means_50_s1, type='l', log='y')
abline(b=0, a=0)
matplot(cbind(pl_means_50_s1[1:590], bs_means_50_s1[1:590]), type='l', log='y')

# avg. records per sec = 215.74973031283707
bs_means_200_s1 <- unlist(lapply(getSimPerfJson("sim1090238404"), function(x) sqrt(x$mean)))

# avg. records per sec =51.49330587023687
bs_means_500_s1 <- unlist(lapply(getSimPerfJson("sim1053370352"), function(x) sqrt(x$mean)))


library(ggplot2)
data = data.frame(
                  "Time"=rep(c(1:1200, 1:1200, 
                                1:1200), 2),
                  "Filter" = c(rep("PL", 3*1200), rep("BS", 3*1200)),
                  "Particles"=c(
                                c(rep("50", 1200), rep("200", 1200), rep("25", 1200)), 
                                c(rep("50", 1200), rep("200", 1200), rep("500", 1200))
                      ),
                  "RMSE"=c(pl_means_50_s1, pl_means_200_s1, pl_means_25_s1, 
                    bs_means_50_s1, bs_means_200_s1, bs_means_500_s1))
data$partFilter = interaction(data$Filter, data$Particles)

pdf(file="plbs-rmse-1200-wisker.pdf")
  #par(mfrow=c(1,1), las=0)
  #lim = 1200
  #pl_means_all = cbind("PL 50 particles"=pl_means_50_s1
  #           ,"PL 200 particles"=pl_means_200_s1
  #           ,"PL 25 particles"=pl_means_25_s1
  #            ,"Bootstrap 50 particles"=bs_means_50_s1
  #            ,"Bootstrap 200 particles"=bs_means_200_s1
  #            ,"Bootstrap 500 particles"=bs_means_500_s1
  #            )
  #boxplot(pl_means_all, log="y", ylab="RMSE")

  p2 <- ggplot(data, aes(partFilter, RMSE)) + geom_boxplot() + scale_y_log10() +
        theme_bw(base_size=10) + scale_x_discrete(name="")                            
  print(p2)

dev.off()


pdf(file="plbs-rmse-1200-series.pdf", pointsize=7) 
  p1 <- ggplot(data, aes(x=Time, y=RMSE, colour=Particles, group=Filter)) +
        geom_line(alpha=0.7) + facet_grid( . ~ Filter) + scale_y_log10() +
        theme_bw(base_size=10)
  plot(p1)

  #plot(cbind("PL 50 particles"=pl_means_50_s1
  #             ,"PL 200 particles"=pl_means_200_s1
  #             ,"PL 500 particles"=pl_means_500_s1
  #              ,"Bootstrap 50 particles"=bs_means_50_s1
  #              ,"Bootstrap 200 particles"=bs_means_200_s1
  #              ,"Bootstrap 500 particles"=bs_means_500_s1
  #              ), 
  #        lty=1,
  #        type="l", log="y", ylab="RMSE")
  #legend("topleft", c("PL 50 particles", 
  #                    "PL 200 particles",
  #                    "PL 500 particles"
  #                    "Bootstrap 50 particles", 
  #                    "Bootstrap 200 particles",
  #                    "Bootstrap 500 particles"
  #                    ),
  #       pch="l", col=c("black", "green", "red", "blue"))
dev.off()

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
                                         
sim_res <- getSimBestStatesJson("sim2076162729")
sim_res <- getSimBestStatesJson("sim1817961029")
sim_res <- getSimBestStatesJson("sim-1200318255")

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

library(ggplot2)

sim_obs_prior_mean_plot <- ggplot(sim_res_obs_prior_mean_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") + scale_y_continuous() + 
      theme_bw(base_size=10)
plot(sim_obs_prior_mean_plot)

sim_obs_cov_plot <- ggplot(sim_res_obs_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") + scale_y_continuous() + 
      theme_bw(base_size=10)
plot(sim_obs_cov_plot)     

sim_obs_qr_plot <- ggplot(sim_res_qr_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") + scale_y_continuous() + 
      theme_bw(base_size=10)
plot(sim_obs_qr_plot)     

sim_obs_qg_plot <- ggplot(sim_res_qg_cov, aes(x=time, y=matrix, colour=type,
                           group=type)) +
      geom_line(alpha=1.0) + facet_grid(row ~ col, scales="free") + scale_y_continuous() + 
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


library("rjson")
json_file <-
  "http://localhost:9000/api/getPerformanceResults?vehicleId=sim-1325570441000"

json_data <- readLines(json_file)
json_data <- sub('[^\\{]*', '', json_data) # remove function name and opening parenthesis
json_data <- sub('\\)$', '', json_data) # remove closing parenthesis

json_data <- fromJSON(paste(json_data, collapse=""))

pl_means_50_s1 <- unlist(lapply(json_data$results, function(x) sqrt(x$mean)))
bs_means_50_s1 <- unlist(lapply(json_data$results, function(x) sqrt(x$mean)))
bs_means_200_s1 <- unlist(lapply(json_data$results, function(x) sqrt(x$mean)))
bs_means_500_s1 <- unlist(lapply(json_data$results, function(x) sqrt(x$mean)))

pdf(file="plbs-rmse-120-50.pdf")
par(mfrow=c(1,1))
#matplot(means, type='l', ylab="RMSE")
#hist(means)
lim = 1200
pl_means_all = cbind(pl_means_10[1:lim], pl_means_50[1:lim],
                     pl_means_100[1:lim], pl_means_200[1:lim])
boxplot(cbind("PL"=pl_means_all), log="y", ylab="RMSE")
#boxplot(cbind("PL"=pl_means_all, "BS"=bs_means[1:lim]), log="y", ylab="Log RMSE")
title("50 particles, 120 observations")
dev.off()

pdf(file="plbs-rmse-1200-series.pdf") 
matplot(cbind("PL 50 particles"=pl_means_50_s1, 
              "Bootstrap 50 particles"=bs_means_50_s1,
              "Bootstrap 200 particles"=bs_means_200_s1,
              "Bootstrap 500 particles"=bs_means_500_s1
              ), 
        lty=1,
        type="l", log="y", ylab="RMSE")
legend("topleft", c("PL 50 particles", 
                    "Bootstrap 50 particles", 
                    "Bootstrap 200 particles",
                    "Bootstrap 500 particles"
                    ),
       pch="l", col=c("black", "green", "red", "blue"))
dev.off()




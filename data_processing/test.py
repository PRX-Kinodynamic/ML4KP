from bokeh.plotting import figure, output_file, show
from bokeh.models import ColumnDataSource
import pandas as pd
import numpy as np

num_sols_found = 0

# output to static HTML file
output_file("planner_output.html",title='DirtMP Output')

def transform_csv(filename,index):
	global num_sols_found
	# df = pd.read_csv(filename,names=["Time","Iters","Nodes","Solution","FoundTime","FoundIter",
	#	"NRBNB","NRP","NRCC","NRA","NBBNB","NBP","NBCC","NBA"],skipfooter=1)
	df = pd.read_csv(filename,names=["Time","Iters","Nodes","Solution","FoundTime","FoundIter",
		"NRBNB","NRP","NRCC","NRA","NBBNB","NBP","NBCC","NBA"])
	num_sols_found = num_sols_found + (df.iloc[-1]['Solution'] != 0)
	df['FirstSolutionIters'] = df["Iters"][df["Solution"].ne(0).idxmax()]
	df['FirstSolutionTime'] = df["Time"][df["Solution"].ne(0).idxmax()]
	df['FirstSolution'] = df["Solution"][df["Solution"].ne(0).idxmax()]
	df['Time'] = pd.to_timedelta(df["Time"], unit='s')
	df = df.set_index(index)
	if index=="Time":
		df = df.drop_duplicates().resample("50L").pad()
	df = df.replace(0, np.nan)
	return df

def augment_df(df,max_index,index):
	if max_index not in df.index:
		new_data = pd.DataFrame(df[-1:].values, index=[max_index], columns=df.columns)
		df = df.append(new_data)
		if index=="Time":
			df = df.resample("50L").pad()
		df = df.replace(0, np.nan)
	df = df.reset_index()
	df.rename(columns = {'index':'Time'}, inplace = True)
	df["Time"] = df["Time"].dt.total_seconds()
	# print df
	return df


data_frames = []

max_index = None

data_dir = "/path/to/planner/data"
planner = "dirt_random"
num_runs = 10

for x in xrange(0,num_runs):
	#df = transform_csv(data_dir+"data_"+planner+"_"+str(x)+".txt","Time")
	# df = transform_csv(data_dir+planner+"_"+str(x)+".txt","Time")
	df = transform_csv(data_dir+str(x)+".txt","Time")
	data_frames.append(df)
	if max_index is None or df.index[-1] > max_index:
		max_index = df.index[-1]
for x in xrange(0,num_runs):
	data_frames[x] = augment_df(data_frames[x],max_index,"Time")

df_concat = pd.concat(data_frames)
by_row_index = df_concat.groupby(df_concat.index)
#df_means = by_row_index.mean()
df_means = by_row_index.mean()
df_means.to_csv("/home/aravind/test.csv")
print("Num solutions found: :", num_sols_found)
source = ColumnDataSource(df_means)

# create a new plot with a title and axis labels
p = figure(title="simple line example", x_axis_label='x', y_axis_label='y')

# add a line renderer with legend and line thickness
p.line(x="Time",y="Solution",source=source, line_width=2)

# show the results
# show(p)
SpheresExperiments
==================

Mike Hopkins free time experiments with SPHERES

RECENTLY:
	-Hardcoded the gains for the estimator function. Most of the gains from before were tailored to individual tests. I found that most of them were the same so I stuck with those values.
	-Added some more todos to the code, take a look at those before loading code on granite table sphere.

TODO:
	-Top priority -- Get the simulator working as soon as possible. I cant know what is wrong with the code until then
	-Finalize the structure for the tests. This may include special modification to spheres core.
	-Investigate more of the code for the glideslope. I need to know how it works.
	-Reintegrate comms into the program so I can debug on the granite table
	-Investigate previous attempts at station keeping. Did they work at all?
	-Set the hardcoded values for the gains outside of the function I set them in. If they are hardcoded there is no reason to be letting them be variable
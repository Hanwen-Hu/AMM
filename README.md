# AMM: An Adaptive Online Map Matching Algorithm

This is the latest version of AMM. the original version is available at https://github.com/kingchiuoy/Ada-Matching. We recommend you to use this one.

All the dependencies have been written in the **pom.xml** file, you can directly use Maven to compile or build the project.

The main function is in "Algorithm/src/main/java/Matching.java", and the source code of AMM is in "Algorithm/src/main/java/AMM".

You can directly run the "Matching.java". The project will output the matching accuracy of 200 tracks from Shanghai. 

**"MatchResult"** saves the matching trajectory of the last track;

**"MatchMap"** saves the maps of Shanghai and Singapore;

**"MatchData"** only contains the Shanghai dataset. The Singapore dataset does not belong to us, so it is not contained in the project.
We open 200 tracks of Shanghai dataset. If you want to use the dataset, please **cite this paper**. 

There are totally 2700 tracks in Shanghai dataset. If you require the rest, please also **cite this paper** and contact us.

Email: hanwen_hu@sjtu.edu.cn

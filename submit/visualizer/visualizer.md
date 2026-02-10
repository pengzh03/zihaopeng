# visualizer

## Installation

### 1. Prerequisites: 

* Python 3.9 or higher.

* The routing cases are ready.

### 2. Install dependencies:

* Navigate to the directory containing `requirements.txt` first.

* Run the following command:
    ~~~
    pip install -r requirements.txt
    ~~~

## Running

> `visualizer1.py` / `visualizer2.py` shows the layout of the target case before / after routing. 

1. Navigate to the directory containing `visualizer1.py` and `visualizer2.py` first.

2. Run the script with the target case name (and total layer count):
    ~~~
    python visualizer1.py [case]
    python visualizer2.py [case] [layer]
    ~~~
    * Mandatory parameter:
      |Name|Type|Description|Requirement|
      |--|--|--|--|
      |`case`|String|Case name (consistent with the case names in the benchmark)|Must be provided|
      |`layer`|String|Total layer count (for visualizer2.py)|Must be provided|
    * Optional parameter:
      |Flag|Type|Description|Requirement|
      |--|--|--|--|
      |`--label`|Flag (no value required), default is `False`|Enable the display of bump names in the visualization|Optional|
      |`--show`|Flag (no value required), default is `False`|Trigger interactive display of the visualization window|Optional|

3. Check the output PDF file: `pic/[case]_grid_visualization_1.pdf` (before routing) and `pic/[case]_grid_visualization_2.pdf` (after routing).

## Other

### kmean visualizer

> `kmean_visualizer.py` shows the layout of the kmean result of target case. 

1. Navigate to the directory containing `kmean_visualizer.py` first.

2. Run the script with the target case name:
    ~~~
    python kmean_visualizer.py [case]
    ~~~
    * Mandatory parameter:
      |Name|Type|Description|Requirement|
      |--|--|--|--|
      |`case`|String|Case name (consistent with the case names in the benchmark)|Must be provided|
    * Optional parameter:
      |Flag|Type|Description|Requirement|
      |--|--|--|--|
      |`--label`|Flag (no value required), default is `False`|Enable the display of bump names in the visualization|Optional|
      |`--show`|Flag (no value required), default is `False`|Trigger interactive display of the visualization window|Optional|

3. Check the output PDF file: `pic/[case]_grid_visualization_kmean.pdf`.

### layer assignment visualizer

> `layer_assignment_visualizer.py` shows the layout of the layer assignment result of target case. 

1. Navigate to the directory containing `layer_assignment_visualizer.py` first.

2. Run the script with the target case name:
    ~~~
    python layer_assignment_visualizer.py [case]
    ~~~
    * Mandatory parameter:
      |Name|Type|Description|Requirement|
      |--|--|--|--|
      |`case`|String|Case name (consistent with the case names in the benchmark)|Must be provided|

3. Check the output PDF file: `pic/[case]_layer_assignment.pdf`.

### build exits visualizer

> `build_exits_visualizer.py` shows the layout of the exits building result of target case. 

1. Navigate to the directory containing `build_exits_visualizer.py` first.

2. Run the script with the target case name:
    ~~~
    python build_exits_visualizer.py [case]
    ~~~
    * Mandatory parameter:
      |Name|Type|Description|Requirement|
      |--|--|--|--|
      |`case`|String|Case name (consistent with the case names in the benchmark)|Must be provided|
    * Optional parameter:
      |Flag|Type|Description|Requirement|
      |--|--|--|--|
      |`--label`|Flag (no value required), default is `False`|Enable the display of bump names in the visualization|Optional|
      |`--show`|Flag (no value required), default is `False`|Trigger interactive display of the visualization window|Optional|

3. Check the output PDF file: `pic/[case]_build_exits.pdf`.
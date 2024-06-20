# lyric-project
Mock implementation(s) for Lyric project presentation

* todo
    * try to figureout a good network flow & or warehouse process optimization approach
        * IB trailer Scheduling as CP solver Flexible Job Shop problem implementation
            * https://github.com/google/or-tools/blob/master/examples/python/flexible_job_shop_sat.py


    * VRP

        * Split demands out to individual nodes so that store capacity can be greater than the route capacity limit
        * Slides
            * problem overview
            * Explain store level batching strategy and the fact that cross store bundles rarely happen do to pay policy
    * IB Scheduling
        * Implement schedule constraint for live unload deliveries
            * https://github.com/google/or-tools/blob/stable/examples/python/jobshop_ft06_distance_sat.py
            * https://github.com/google/or-tools/blob/stable/examples/python/single_machine_scheduling_with_setup_release_due_dates_sat.py
        * Slides
            * 
            * Give over view of ML models and training strategies for core productivity predictions
                * yarn move time & unload time
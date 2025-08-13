import h5py
import numpy as np
from corridor_utils import plot_trajectory_and_corridor

def read_all_trials(file_path):

    trials = []


    with h5py.File(file_path, "r") as f:
        print("Scanning HDF5 file...")

        # Walk through all groups
        for group_name, group in f.items():
            if not isinstance(group, h5py.Group):
                continue

            print(f"\n🔹 Group: /{group_name}")

            # Try to load route
            try:
                route = np.array(group["route"])
                print(f" route: shape {route.shape}")
            except KeyError:
                print("  no route")

            # Try to load polys
            polys_list = []


            if "polys" in group and isinstance(group["polys"], h5py.Group):
                polys = group["polys"]
                print(f" polys group with {len(polys)} entries:")
                for poly_name, poly_dset in polys.items():
                    data = np.array(poly_dset)
                    #print(f"    - {poly_name}: shape {data.shape}")
                    #print(poly_dset[()])  # Print the actual data
                    polys_list.append(data)
            else:
                print("  no polys group")

            trials.append({
                "route": route,
                "polys": polys_list
            })


    return trials


if __name__ == "__main__":
    # Change path to your file
    trials = read_all_trials("../dataset/dataset_10.h5")


    first_trial = trials[0]
    trajectory = first_trial["route"]
    corridors = first_trial["polys"]

    plot_trajectory_and_corridor(trajectory, corridors)

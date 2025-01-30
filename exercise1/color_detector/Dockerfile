FROM duckietown/dt-duckiebot-interface:daffy-arm64v8
# ^ This is the correct base image for Duckiebot MOOC Founder's Edition (ARM64).

# Optionally fix the GPG key issue if you ever need apt-get inside the container:
# RUN apt-get update && apt-get install -y curl gnupg2 && \
#     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
#     apt-get update && apt-get install -y libgl1-mesa-glx && \
#     rm -rf /var/lib/apt/lists/*

# Set the working directory
WORKDIR /color_detector_dir

# Copy your entire project into the container (includes color_detector.py and requirements.txt)
COPY . .

# Install Python dependencies (assuming the base image already has Python and pip)
RUN pip install --no-cache-dir -r requirements.txt

# (Optional) Set an environment variable
ENV N_SPLITS=1

# Define the default command to run when the container starts
CMD ["python3", "color_detector.py"]
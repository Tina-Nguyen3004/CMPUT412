FROM duckietown/dt-duckiebot-interface:daffy-arm32v7 
# # use daffy-arm64v8 if you are using a Duckiebot MOOC Founder's Edition

# WORKDIR /color_detector_dir

# COPY requirements.txt ./

# RUN pip install -r requirements.txt

# COPY color_detector.py .

# CMD python3 ./color_detector.py

# Use a base image with Python
FROM python:3.12

# Set the working directory
WORKDIR /color_detector_dir

# Install required system dependencies
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

# Copy the project files
COPY . .

# Install dependencies from requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Set environment variable (optional)
ENV N_SPLITS=1

# Run the script when the container starts
CMD ["python3", "color_detector.py"]
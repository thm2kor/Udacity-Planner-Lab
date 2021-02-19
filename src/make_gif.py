import imageio
import os
image_folder="images"
down_sample=1
images = []
files = [x for x in os.listdir(image_folder) if x.endswith('.jpg')]
files.sort()
files = [files[i] for i in range(0,len(files),down_sample)]
for file in files:
  images.append(imageio.imread(os.path.join(image_folder,file)))
images.append(imageio.imread(os.path.join(image_folder,files[-1])))
images.append(imageio.imread(os.path.join(image_folder,files[-1])))
images.append(imageio.imread(os.path.join(image_folder,files[-1])))


imageio.mimsave(image_folder + '.gif', images, duration=0.3)
print("Image is saved to ",image_folder + '.gif')

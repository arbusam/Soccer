# Tech Support 2026

We are Tech Support, a robotics team from Melbourne High School (MHS) competing in the RoboCup Junior Australia (RCJA) Soccer Open competition. This repo contains all the code used for our robots.

This repo was created to share our code to provide inspiration to other teams. However, as stated in the [RCJA General Rules](https://www.robocupjunior.org.au/wp-content/uploads/2026/02/2026-RCJA-General-Rules.pdf) in Rule 4.3.2:
> Teams may not directly use designs and programs that has been passed down to them by the teams before them

## Code Structure

TODO: Structure the code and fill this in

`training/` contains all the code used to train our YOLO26n model on my hardware. I do not recommend using this code as it was only used to train a very specific model with a very specific setup on my specific computer.

The models themselves are stored in `open-soccer-detect-n_hailo_model` and `open-soccer-detect-s_hailo_model`. These are trained models based on YOLO26n and YOLO26s respectively, and quantised to be run on a Raspberry Pi AI HAT. The training data used for these models can be found here (TODO: Upload training data), however, it is also not recommended to use this data as you would likely get better results using data captured on the same hardware that will be used during gameplay, and as mentioned at the top, would be a breach of the RCJA rules.

## AI Declaration

Generative AI tools such as GitHub Copilot and Cursor were used to assist in writing the code in this repo. We believe in using AI to enhance human development, not replace it, so in accordance with the MHS Robotics Club Generative AI policy, all essential code (such as `main.py`, `defence.py`, etc) has been verified by our team. However, test code and code used for model training was mostly written by AI, as it is not used during gameplay.

## Contact

If you would like to contact us please use the following contact details:

Arhan - Email: [arhan@arhan.tech](mailto:arhan@arhan.tech) Discord: @arbusam
Kanishk - Email: @kansar_1

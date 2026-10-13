import passport from "passport";
import { Strategy as GoogleStrategy, Profile } from "passport-google-oauth20";
import { config } from "./env";

export interface GoogleProfile {
  googleId: string;
  email: string;
  name: string;
  avatarUrl?: string;
}

function mapGoogleProfile(profile: Profile): GoogleProfile {
  const email = profile.emails?.[0]?.value;

  if (!email) {
    throw new Error("Google profile did not include an email address");
  }

  return {
    googleId: profile.id,
    email: email.toLowerCase(),
    name: profile.displayName,
    avatarUrl: profile.photos?.[0]?.value,
  };
}

passport.use(
  new GoogleStrategy(
    {
      clientID: config.google.clientId,
      clientSecret: config.google.clientSecret,
      callbackURL: config.google.callbackUrl,
    },
    (_accessToken, _refreshToken, profile, done) => {
      try {
        done(null, mapGoogleProfile(profile) as unknown as Express.User);
      } catch (err) {
        done(err as Error);
      }
    },
  ),
);

export { passport };

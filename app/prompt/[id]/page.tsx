import type {Metadata} from "next";
import {notFound} from "next/navigation";
import PromptDetail from "@/components/PromptDetail";
import {createClient} from "@/lib/supabase/server";
import type {Prompt} from "@/types";
export const dynamic="force-dynamic";
function active(sub:{status?:string|null;current_period_end?:string|null}|null){return sub?.status==="active"&&(!sub.current_period_end||new Date(sub.current_period_end).getTime()>Date.now())}
export async function generateMetadata({params}:{params:{id:string}}):Promise<Metadata>{const supabase=createClient();const {data}=await supabase.from("prompts").select("title").eq("id",params.id).maybeSingle();return {title:data?.title?`${data.title} - PromptBox`:"PromptBox"}}
export default async function Page({params}:{params:{id:string}}){const supabase=createClient();const {data:prompt,error}=await supabase.from("prompts").select("*").eq("id",params.id).maybeSingle();if(error||!prompt)notFound();const {data:{user}}=await supabase.auth.getUser();let isPro=false;let isFavorited=false;if(user){const {data:sub}=await supabase.from("subscriptions").select("status,current_period_end").eq("user_id",user.id).maybeSingle();isPro=active(sub);const {data:fav}=await supabase.from("user_favorites").select("id").eq("user_id",user.id).eq("prompt_id",params.id).maybeSingle();isFavorited=Boolean(fav)}return <PromptDetail prompt={prompt as Prompt} isPro={isPro} isFavorited={isFavorited}/>}
